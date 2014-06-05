/****************************************************************************
 *
 * IP.ACCESS -
 *
 * Copyright (c) 2010 ip.access Ltd.
 *
 ****************************************************************************
 *
 * File Description : keystream driver using an mmap interface
 *
 * Control and status is via ioctl calls.  Keystream data and status is made
 * available via mmap().  Crypto engine HW DMAs directly into the mmap area,
 * so care must be taken to ensure that the CPU and DMA don't overwrite each
 * other's output.
 *
 * The cipher data is passed back to the client in the mmap'ed area of memory.
 * This memory is kmalloc'ed by the driver, so it is contiguous in physical
 * memory.  That allows a simple mapping between the kernel's addresses and
 * the hardware addresses used by the cipher engine's DMA hardware.
 *
 * Requests for cipher data are passed to the driver via an ioctl request.
 * Each request specifies the number and size of the PDUs (all the PDUs in
 * a single request are the same size) and supplies a count value for each
 * PDU.  The count is used to form the Initialisation Vector when generating
 * the cipher data.
 *
 * Part of the mmap area is allocated to a request (wrapping when necessary).
 * Each request starts with a header of 32 bytes (one cache line) that holds
 * information to pass back to the caller (pduLength, contextId etc).  The
 * header is then followed by a block of keystream data that's rounded up to
 * a multiple of 32 bytes to ensure the next header is still cache aligned.
 *
 * When a request is made, the cipher engine may be able to take all the PDUs
 * making up the request immediately.  If so, the PDUs are passed straight to
 * the cipher engine.  As PDUs are sumbitted, a pointer to the request is
 * saved to the head of a circular buffer of request pointers.  As cipher
 * requests complete, the associated request is read from the tail of the
 * list and the request's count of completed PDUs is updated.  Once all the
 * PDUs for a single request have been completed, the request can be returned
 * to the caller.
 *
 * If the cipher engine is not able to take all the PDUs, then they are queued.
 * The request pointer is saved at the head of a circular buffer of requests
 * and the count values associated with each PDU are also saved on a (bigger)
 * circular buffer.
 *
 * The cipher engine has a limit of 128 simultaneuous cipher operations.
 * We submit entire requests at once and each request is typically 17 PDUs,
 * each of 80-bytes.  That gives a maximum of 7 requests == 119 PDU cipher
 * operations submitted.  With queueing, we can add another 60 requests of
 * 17 PDUs since we can save up to 1024 count values, one per PDU.  Since
 * the mmap'ed area is currently 64K this should be enough, since the total
 * memory that would be needed is (60+7 requests) * 17 PDUs * 80 bytes ~=
 * 91K.  So the lowest limit is currently the mmap memory size.  12.5Mbps
 * will give 64K in 41ms, so we have to be able to get the data in and out
 * in less than about 4 * 10ms ticks.
 *
 * Whenever the client app submits more data, or polls for completed requests,
 * the driver will check for completed cipher operations.  If that has made
 * enough room for the next request on the queue then that will be submitted.
 * The request and the counts for the PDUs are read from the circular buffer
 * and submitted to the cipher engine.
 *
 * In addition to this polling, the cipher engine is configured to generate
 * an interrupt if there are more than 64 = (128/2) completed requests.
 * The interrupt ensures that the cipher engine will keep itself busy for
 * as long as there is queued data; the app does not need to keep polling
 * just to process the queued backlog.
 *
 * Because we modify the mmap'ed region via DMA, we have to be careful how
 * we handle cached data.  Normally, such a region should be marked as being
 * non-cacheable from the user app side.  However, this has a large hit on
 * the performance.  If the data cannot be cached, each 32-bit (or even
 * worse 8-bit) read of the cipher data requires a separate SDRAM access.
 * To avoid this the mmap'ed region is marked as cacheable and the app will
 * be able to do burst reads.  We have to be careful here though as a result.
 * Whenever we write to the request header using a kernel address, we must
 * flush the cache line to memory.  Similarly, after the app has read the
 * cipher data from a request we must force it to drop any data it has saved
 * in its cache to ensure that it sees the changed data for the next request.
 * We do this when the app tells us it has finished with the data via the
 * buffer unload ioctl.
 *
 ****************************************************************************/


/****************************************************************************
 * Standard Library Includes
 ****************************************************************************/
#include <linux/clk.h>
#include <linux/cdev.h>
#include <linux/configfs.h>
#include <linux/crypto.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/scatterlist.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/sysfs.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/version.h>
#include <asm/pgtable.h>
#include <asm/cacheflush.h>

#include <mach/hardware.h>

/****************************************************************************
 * Project Includes
 ****************************************************************************/
#include <linux/ipa/ipa_keystream.h>
#include <../drivers/crypto/picoxcell_crypto_regs.h>


/****************************************************************************
 * Private Definitions
 ****************************************************************************/
#define IPAC_DMA_HW_ALIGNMENT            0x00010000
#define IPAC_DMA_HW_PAGE_MASK            0xFFFF0000
#define IPAC_DMA_HW_OFFSET_MASK          0x0000FFFF

#define ALLOW_CACHED_USER_READS 

#define IPAC_DEVICE_MAJOR  242

/* Each context block in the cipher engine memory map takes up this
 * number of bytes.
 */
#define IPAC_L2_CIPHER_CONTEXT_SIZE          64

/* Number of PDUs that can be queued for the cipher engine HW at once.
 * The cipher engine has a couple of FIFOs internally that are this size.
 * They're used to queue up requests inwards and results outwards.
 */
#define IPAC_NUM_CIPHER_ENGINE_REQUESTS      128

/* We keep a circular buffer of pointers to all the requests that have been
 * queued because the cipher engine is too busy.  We expect to have no more
 * than about 32 requests per 5ms tick.  So 256 should be more than enough
 * to handle 64 requests over a 10ms tick.
 *
 * MUST BE A POWER OF TWO
 */
#define IPAC_MAX_NUM_QUEUED_REQUESTS         256

/* If we had 64 requests in 10ms and each had 17 PDUs then we'd have 1088 PDUs.
 * 128 of those could be sent directly to the HW, so we should never have to
 * queue more than 1024 PDUs.  We'd not have enough request memory anyway for
 * 1024*80 byte PDUs since there's only 64K allocated for passing cipher stream
 * data back to the user app.
 *
 * MUST BE A POWER OF TWO
 */
#define IPAC_MAX_NUM_QUEUED_COUNTS           2048

/* This is the value that's stored in the key size register.  Once it's
 * been calculated once, we can get the context value for the next index
 * by adding an increment.
 */
#define IPAC_KEY_SZ_VALUE(idx)  ( IPAC_KEY_BYTES                        | \
                                  ( 1 << SPA_KEY_SZ_CIPHER_OFFSET )     | \
                                  ((idx) << SPA_KEY_SZ_CTX_INDEX_OFFSET) )
                                
#define IPAC_KEY_SZ_INCREMENT   (1 << SPA_KEY_SZ_CTX_INDEX_OFFSET)
                                  
/* This is the value that's stored in the control register.  Once it's
 * been calculated once, we can get the control register value for the
 * next index by adding an increment.
 */
#define IPAC_CTRL_VALUE(idx)    ( SPA_CTRL_CIPH_ALG_KASUMI    | \
                                  SPA_CTRL_CIPH_MODE_F8       | \
                                  (1 << SPA_CTRL_ENCRYPT_IDX) | \
                                  ((idx) << SPA_CTRL_CTX_IDX ) )
                                
#define IPAC_CTRL_VALUE_INCREMENT  (1 << SPA_CTRL_CTX_IDX )


#define IPAC_CIPHER_KEY_VERSION       2

/* We support on f8(kasumi) in this driver */
#define IPAC_CIPHER_KEY_LENGTH_BITS   128
#define IPAC_IV_LENGTH_BITS           64

#define IPAC_KEY_BYTES                ((IPAC_CIPHER_KEY_LENGTH_BITS + 7) / 8)
#define IPAC_KEY_LONGS                ((IPAC_CIPHER_KEY_LENGTH_BITS + 31) / 32)
#define IPAC_IV_LENGTH_BYTES          ((IPAC_IV_LENGTH_BITS + 7) / 8)
#define IPAC_IV_LENGTH_LONGS          ((IPAC_IV_LENGTH_BITS + 31) / 32)

/* Allow optional debugging output to be enabled */
#define DEBUG_BASIC   ( DBG_ALWAYS | DBG_ERROR | DBG_WARN | DBG_NOTICE )
#define DEBUG_MASK    ( DEBUG_BASIC )

#define PRINTD( _mask, _fmt, ... ) \
    ({ \
        if ( _mask & DEBUG_MASK ) \
            printk( "%s:%u: " _fmt "\n", __FILE__, __LINE__, ##__VA_ARGS__ ); \
    })


/****************************************************************************
 * Private Types
 ****************************************************************************/
typedef enum   IpacDirectionTag     IpacDirection;

typedef struct IpacDeviceTag        IpacDevice;
typedef struct IpacDeviceNodeTag    IpacDeviceNode;
typedef struct IpacContextTag       IpacContext;
typedef struct IpacDdtContextTag    IpacDdtContext;
typedef struct IpacDdtTag           IpacDdt;

enum IpacDebugOptions
{
    DBG_ALWAYS     =  (1 <<  0),  /* Used for getting messages during development*/
    DBG_ERROR      =  (1 <<  1),  /* Critical errors. */
    DBG_WARN       =  (1 <<  2),  /* Warning messages. */
    DBG_NOTICE     =  (1 <<  3),  /* Notices of abnormal conditions that the driver should handle. */
    DBG_FUNC_CALLS =  (1 <<  4),  /* Tracing of function entry and exit. */
    DBG_BUFFER     =  (1 <<  5),  /* Request buffer handling. */
    DBG_MEM_CACHE  =  (1 <<  6),  /* Track requests use of the mem cache */
    DBG_CIPHER     =  (1 <<  7),  /* Track creation and deletion of cipher contexts */
    DBG_CIPHER_REQ =  (1 <<  8),  /* Track cipher requests */
    DBG_MMAP       =  (1 <<  9),  /* Track mmap operations */
    DBG_SCATTER    =  (1 << 10),  /* Track scatter-gather */
    DBG_PDU_COUNT  =  (1 << 11),  /* Track counting of PDUs within a request */
    DBG_IO         =  (1 << 12),  /* Track cipher engine register accesses */
    DBG_PDU_QUEUE  =  (1 << 13),  /* Track count values as they're queued and unqueued */
    DBG_IRQ        =  (1 << 14),  /* Track interrupt calls */
    DBG_DMA_ALIGN  =  (1 << 15),  /* Track behaviour of the DMA alignment function */
};

enum IpacDirectionTag
{
    IPAC_UPLINK,
    IPAC_DOWNLINK
};

/* We have an array of contexts.  A context can hold two keys, the current one
 * and the previously used one.  That allows requests to complete while a new
 * request is submitted with an updated key.
 */
struct IpacContextTag
{
    /* Ensure key data is aligned on a 32 bit boundary */
    u32        keyData[2][IPAC_KEY_LONGS];
    
    /* The counts kept in cipherActiveRequests and oldCipherActiveRequests and the
     * nextkeyBuffer pointer will be changed by the IRQ service routine.  We should
     * ensure that any accesses to them are protected by the IRQ spinlock for the
     * HW device, not just the device node lock.
     *
     * The keys themselves should only be modified by user space functions so they can be
     * protected by the device node lock.
     */
    u32       *key_p;                     /* Pointer to active key used for new requests */
    u32        cipherActiveRequests;      /* How many requests have been submitted with the current cipher */
    u32        ivFixedPart;               /* The combination of binding ID and direction */
    
    u32       *oldKey_p;                  /* Pointer to old key used for old requests when switching */
    u32        oldCipherActiveRequests;   /* How many requests have still to finish with the old cipher */
    
    u8         nextKeyBuffer;
};

/* The driver supports a number of device nodes that are held in an array.
 * Information specific to each node is held in the following structure.
 * There's usually one node for all uplink and another for all downlink.
 */
struct IpacDeviceNodeTag
{
    u8                 name[16];                    /* A name for the node */
    spinlock_t         lock;                        /* Lock to allow callbacks and user space calls */
    int                useCount;                    /* How many clients have opened this node (max one allowed) */
    IpacDirection      direction;                   /* Whether this node is for uplink or downlink */
    struct device      *dev_p;                      /* Pointer to the device created for this node */
    u8                 *vm_start;                   /* The start of the memory buffer as seen by the user */
    u8                 *startOfPool_p;              /* The start of the memory buffer as seen by the kernel */
    u8                 *endOfPool_p;                /* One past the end of the memory buffer */
    u8                 *firstRequest_p;             /* Pointer to the first (oldest) request */
    u8                 *lastRequest_p;              /* Pointer to a marker after the last (newest request) */
    dma_addr_t         startOfDma_p;                /* The start of the memory buffer as mapped for DMA */
    u32                dmaOffset;
    u32                requestsPending;             /* The number of requests that the user has waiting */
    u8                 releasingNode;               /* Flag to indicate that the client is closing */
    u8                 inIoctlCall;                 /* Flag to indicate that the client task is running IOCTL */
    IpacContext        context[IPAC_NUM_CONTEXTS];  /* Cipher contexts for this node */
    u32                tempCounts[IPAC_MAX_NUM_PDUS];
};
 

/* All transfers are a single contiguous block, so we just need the
 * address and length of the block (p0, len0) and a zeroed terminator
 * (which can be set once at startup) in p1, len1.
 */
struct IpacDdtTag
{
    u32  p0;
    u32  len0;
    u32  p1;
    u32  len1;
};

#define __ddt_align             __attribute__((aligned(16)))

struct IpacDdtContextTag
{
    IpacDdt  __ddt_align ddt;
};

/* Info about the driver/device.  This is mainly status associated with the
 * cipher engine operations.
 */
struct IpacDeviceTag {
    struct cdev         cdev;
    dev_t               devno;
    struct class       *sysfs_class;
    
    struct clk         *clk;                /* The clock source that enables/disabled the cipher engine */
    struct device      *dev;
    void __iomem       *crypto_regs;        /* Pointer to the cipher control/status registers */
    void __iomem       *cipher_ctx_base;    /* Pointer to the cipher context area */
    void __iomem       *hash_key_base;      /* Pointer to the cipher hash area (not used for F8) */
    IpacDdtContext     *dst_ddt_buf;        /* Pointer to an array of DDTs pointing to memory for cipher results */
    IpacDdtContext     *src_ddt_buf;        /* Pointer to a DDT pointing to zeroed memory for cipher input */
    dma_addr_t          dst_ddt_buf_phys;   /* Physical address of destination DDTs */
    dma_addr_t          src_ddt_buf_phys;   /* Physical address of source DDT */
    
    void               *zeroed_page;        /* Pointer to zeroed page of memory */
    struct scatterlist  zeroed_list;        /* Scatter list mapping the zeroed page */

    spinlock_t          hw_lock;            /* There are two nodes (up/down), but only one engine, so we need to lock it */
    
    /* Maintain a circular buffer of PDU requests that have been sent to the engine
    * so we can match the results with the requests.  Since we have as many buffer
    * positions as there are cipher engine contexts and queued requests, we can use
    * the activeRequestHead as the context index too.
    */
    u32                 activeRequestHead;  /* Accesses should always be protected by IRQ spinlock */
    u32                 activeRequestTail;  /* Accesses should always be protected by IRQ spinlock */
    u32                 numActiveRequests;  /* Accesses should always be protected by IRQ spinlock */
    
    /* Maintain a circular buffer of requests that had to be deferred because the
    * cipher HW was busy.  As cipher requests complete, we'll submit these.
    */
    u32                 queuedRequestHead;  /* Accesses should always be protected by IRQ spinlock */
    u32                 queuedRequestTail;  /* Accesses should always be protected by IRQ spinlock */
    u32                 numQueuedRequests;  /* Accesses should always be protected by IRQ spinlock */
    
    /* Maintain a circular buffer of count values.  Each queued PDU has one of these
    * associated with it.  The count is used as part of the IV for the cipher
    * operation.
    */
    u32                 queuedCountHead;  /* Accesses should always be protected by IRQ spinlock */
    u32                 queuedCountTail;  /* Accesses should always be protected by IRQ spinlock */
    u32                 numQueuedCounts;  /* Accesses should always be protected by IRQ spinlock */
    
    IpacRequest        *activeRequests[IPAC_NUM_CIPHER_ENGINE_REQUESTS];   /* Accesses should always be protected by IRQ spinlock */
    IpacRequest        *queuedRequests[IPAC_MAX_NUM_QUEUED_REQUESTS];      /* Accesses should always be protected by IRQ spinlock */
    u32                 queuedRequestsDone[IPAC_MAX_NUM_QUEUED_REQUESTS];  /* Accesses should always be protected by IRQ spinlock */
    u32                 queuedCounts[IPAC_MAX_NUM_QUEUED_COUNTS];          /* Accesses should always be protected by IRQ spinlock */
};


/****************************************************************************
 * Private Function Prototypes (Must be declared static)
 ****************************************************************************/

/* Callback functions for cipher */
static void ipacCipherComplete( IpacRequest *request_p, int err );
static void ipacProcessCompletedPdus( void );
static void ipacProcessQueuedPdus( void );

/* Module operations */
static int ipacRelease( struct inode *inode, struct file *filp );
static int ipacOpen( struct inode *inode, struct file *filp );
static int ipacMmap( struct file *filp, struct vm_area_struct *vma );
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
static long ipacIoctl( struct file *filp, unsigned int cmd, unsigned long arg );
#else
static int ipacIoctl( struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg );
#endif
static ssize_t ipacRead( struct file *filp, char __user *buf, size_t buf_len, loff_t *offset );

/* Mmap operations */
static void ipacVmaOpen( struct vm_area_struct *vma );
static void ipacVmaClose( struct vm_area_struct *vma );
static int ipacVmaFault( struct vm_area_struct *vma, struct vm_fault *vmf );

/****************************************************************************
 * Private Constants (Must be declared static)
 ****************************************************************************/


/****************************************************************************
 * Exported Variables
 ****************************************************************************/

 
/****************************************************************************
 * Private Variables (Must be declared static)
 ****************************************************************************/
 
static struct file_operations ipacFileOps = {
    .owner    = THIS_MODULE,
    .open     = ipacOpen,
    .release  = ipacRelease,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
    .unlocked_ioctl = ipacIoctl,
#else
    .ioctl    = ipacIoctl,
#endif
    .read     = ipacRead,
    .mmap     = ipacMmap,
};

static struct vm_operations_struct ipacNopageVmOps = {
    .open     = ipacVmaOpen,
    .close    = ipacVmaClose,
    .fault    = ipacVmaFault,
};


static IpacDevice      ipacDevice;
static IpacDeviceNode  ipacNodes[IPAC_MAX_DRIVER_NODES];


/******************************************************************************
 * Function Name : ipacRegWrite
 * Inputs        : offset - Offset of the cipher engine register to write to.
 *                 value  - value to write to the register.
 * Outputs       : -
 * Returns       : -
 *
 * Description   : This function will write a 32 bit value to a HW register.
 *
 *****************************************************************************/
static inline void ipacRegWrite( unsigned long  offset,
                                 u32            value )
{
    void __iomem  *addr = ipacDevice.crypto_regs + offset;
    
    PRINTD( DBG_IO, "write %x to register at offset %lx", value, offset );
    
    iowrite32( value, addr );
}

/******************************************************************************
 * Function Name : ipacRegRead
 * Inputs        : offset - Offset of the cipher engine register to read from.
 * Outputs       : -
 * Returns       : value read
 *
 * Description   : This function will read a 32 bit value from a HW register.
 *
 *****************************************************************************/
static inline u32 ipacRegRead( unsigned long  offset )
{
    void __iomem  *addr = ipacDevice.crypto_regs + offset;
    u32            val  = ioread32( addr );
    
    PRINTD( DBG_IO, "read %x from register at offset %lx", val, offset );
    
    return val;
}

/******************************************************************************
 * Function Name : ipacFlushRequestInfo
 * Inputs        : request_p - Pointer to request header to flush.
 * Outputs       : -
 * Returns       : -
 *
 * Description   : This function will clean a single request header from the
 *                 data cache to the SDRAM.  If the user space is accessing the
 *                 request headers then this is necessary to ensure that any
 *                 updates made in kernel space are seen in user space.
 *
 *****************************************************************************/
static void ipacFlushRequestInfo( const IpacRequest  *request_p )
{
    u8  *start_p = (u8*)request_p;
    u8  *end_p   = ((u8*)(request_p + 1)) - 1;

    PRINTD(DBG_FUNC_CALLS, "ipacFlushRequestInfo(%p)", request_p);
 
    dmac_map_area(start_p, end_p-start_p, DMA_TO_DEVICE);
    
    PRINTD(DBG_FUNC_CALLS, "ipacFlushRequestInfo out");
}

#ifdef ALLOW_CACHED_USER_READS 
/******************************************************************************
 * Function Name : ipacInvalidateCipherData
 * Inputs        : start_p - Pointer to first byte to invalidate
 *                 end_p   - Pointer to last byte to invalidate
 * Outputs       : -
 * Returns       : -
 *
 * Description   : This function will invalidate the cache line(s) holding a
 *                 cipher stream.  That ensures that the user app can see new
 *                 data, but still get the advantages of burst reads via the
 *                 data cache.
 *
 *****************************************************************************/
static void ipacInvalidateCipherData( u8*  start_p,
                                      u8*  end_p )
{
    PRINTD(DBG_FUNC_CALLS, "ipacInvalidateCipherData(%p, %p)", start_p, end_p);
 
    dmac_map_area(start_p, end_p-start_p, DMA_FROM_DEVICE);
    
    PRINTD(DBG_FUNC_CALLS, "ipacInvalidateCipherData out");
}
#endif

/******************************************************************************
 * Function Name : ipacSetupContext
 * Inputs        : context_p   - Pointer to the context to configure.
 *                 bearerId    - bearer ID associated with the cipher context
 *                 direction   - uplink/downlink 
 *                 saveCurrent - whether to backup existing cipher info
 *                 keyData_p   - Pointer to the key to associate with the
 *                               context.
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Setup a context for a supplied cipher.  If necessary the
 *                 existing cipher will be backed up so that requests that
 *                 are already queued will be able to complete.
 *
 *****************************************************************************/
static void ipacSetupContext( IpacContext  *context_p,
                              u32           bearerId,
                              u32           direction,
                              int           saveCurrent,
                              u8           *keyData_p )
{
    unsigned long  flags;

    PRINTD(DBG_FUNC_CALLS, "ipacSetupContext(%p, %u, %u, %d, %p)",
                            context_p, bearerId, direction, saveCurrent, keyData_p);
 
    context_p->ivFixedPart = htonl(((bearerId & 0x1F) << 27) | ((direction & 1) << 26));
    
    spin_lock_irqsave( &ipacDevice.hw_lock, flags );

    if (saveCurrent)
    {
        /* Keep the old cipher around for a while if there are requests that 
         * still have to be actioned with the current cipher.
         */
        if (context_p->cipherActiveRequests)
        {
            PRINTD(DBG_CIPHER, "Backing up cipher key %p to oldCipher", context_p->key_p);
            context_p->oldKey_p                = context_p->key_p;
            context_p->oldCipherActiveRequests = context_p->cipherActiveRequests;
        }
        else
        {
            PRINTD(DBG_CIPHER, "Freeing cipher key %p; update with no clients", context_p->key_p);
            context_p->oldKey_p                = NULL;
            context_p->oldCipherActiveRequests = 0;
        }
    }
    
    context_p->key_p                = context_p->keyData[context_p->nextKeyBuffer];
    context_p->cipherActiveRequests = 0;
    context_p->nextKeyBuffer        = !context_p->nextKeyBuffer;
    
    memcpy(context_p->key_p, keyData_p, IPAC_KEY_BYTES);

    spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );

    PRINTD(DBG_CIPHER, "Setup cipher key %p", context_p->key_p);
    
    PRINTD(DBG_FUNC_CALLS, "ipacSetupContext out");
}

/******************************************************************************
 * Function Name : ipacAllocateContext
 * Inputs        : filp    - file info associated with user's file descriptor
 *                 ioctl_p - Pointer to the context info passed by the user.
 * Outputs       : -
 * Returns       : context ID or -ve error code
 *
 * Description   : Look for a free context and allocate it to the user.
 *                 Configure the context with the data passed by the user.
 *
 *****************************************************************************/
static int ipacAllocateContext( struct file       *filp, 
                                IpacContextIoctl  *ioctl_p )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    int              ret       = -EBUSY;
    int              contextId;
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateContext(%p, %p)", filp, ioctl_p);

    spin_lock_bh( &node_p->lock );
    
    /* Search for an unused context (one that doesn't have an associated key) */
    for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
    {
        IpacContext *context_p = &node_p->context[contextId];
        
        if (context_p->key_p == NULL)
        {
            PRINTD(DBG_CIPHER, "Allocate cipher context %d/%p", contextId, context_p);
            ipacSetupContext(context_p, ioctl_p->bearerId, node_p->direction, 0, &ioctl_p->cipherKey[0]);
            ret = contextId;
            break;
        }
    }

    spin_unlock_bh( &node_p->lock );
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateContext returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacDeallocateContext
 * Inputs        : filp      - file info associated with user's file descriptor
 *                 contextId - Index of the context to be freed
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Free a context previously allocated to the user.
 *
 *****************************************************************************/
static int ipacDeallocateContext( struct file  *filp,
                                  int           contextId )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacContext     *context_p = NULL;
    int              ret       = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacDeallocateContext(%p, %d)", filp, contextId);
 
    if ((contextId < 0) || (contextId >= IPAC_NUM_CONTEXTS))
    {
        PRINTD(DBG_NOTICE, "Invalid contextId %d", contextId);
        ret = -EINVAL;
    }
    else
    {
        unsigned long flags;

        spin_lock_bh( &node_p->lock );
    
        spin_lock_irqsave( &ipacDevice.hw_lock, flags );

        context_p = &node_p->context[contextId];
    
        if (context_p->key_p == NULL)
        {
            PRINTD(DBG_NOTICE, "No cipher associated with context to be deallocated");
            ret = -ENOENT;
        }
        else if ((context_p->oldKey_p != NULL) && (context_p->cipherActiveRequests != 0))
        {
            PRINTD(DBG_NOTICE, "Two ciphers running for (%d/%p), can't deallocate.", contextId, context_p);
            ret = -EBUSY;
        }
        else
        {
            /* Deactivate current cipher, but watch for existing requests */
            if (context_p->cipherActiveRequests)
            {
                PRINTD(DBG_CIPHER, "Saving cipher key %p as oldCipher", context_p->key_p);
                context_p->oldKey_p                = context_p->key_p;
                context_p->oldCipherActiveRequests = context_p->cipherActiveRequests;
                context_p->nextKeyBuffer           = !context_p->nextKeyBuffer;
            }
            else
            {
                PRINTD(DBG_CIPHER, "Releasing cipher key %p ", context_p->key_p);
            }
        
            context_p->key_p                = NULL;
            context_p->cipherActiveRequests = 0;
            
            PRINTD(DBG_CIPHER, "Deallocate cipher context %d/%p", contextId, context_p);
            
            ret = 0;
        }
    
        spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );
    
        spin_unlock_bh( &node_p->lock );
    }

    PRINTD(DBG_FUNC_CALLS, "ipacDeallocateContext returned %d", ret);
 
    return ret;
}

/******************************************************************************
 * Function Name : ipacUpdateContext
 * Inputs        : filp    - file info associated with user's file descriptor
 *                 ioctl_p - Pointer to the context info passed by the user.
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Update a context with the information passed by the user.
 *
 *****************************************************************************/
static int ipacUpdateContext( struct file       *filp,
                              IpacContextIoctl  *ioctl_p )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    int              contextId = ioctl_p->contextId;
    IpacContext     *context_p = NULL;
    int              ret       = -EINVAL;
    
    PRINTD(DBG_FUNC_CALLS, "ipacUpdateContext(%p, %p)", filp, ioctl_p);
 
    if ((contextId < 0) || (contextId >= IPAC_NUM_CONTEXTS))
    {
        PRINTD(DBG_NOTICE, "Invalid contextId %d", contextId);
        ret = -EINVAL;
    }
    else
    {
        spin_lock_bh( &node_p->lock );
    
        context_p = &(node_p->context[contextId]);
    
        if ((context_p->key_p != NULL) && (context_p->oldKey_p != NULL))
        {
            /* Both in use */
            PRINTD(DBG_NOTICE, "contextId %d is too busy", contextId);
            ret = -EAGAIN;
        }
        else if (context_p->key_p != NULL)
        {
            /* Current is in use, move to old */
            ipacSetupContext(context_p, ioctl_p->bearerId, node_p->direction, 1, &ioctl_p->cipherKey[0]);
            PRINTD(DBG_CIPHER, "Update cipher context %d/%p", contextId, context_p);
            ret = contextId;
        }
        else
        {
            /* Current is not in use, can't update */
            PRINTD(DBG_NOTICE, "contextId %d is not in use", contextId);
            ret = -ENOENT;
        }
    
        spin_unlock_bh( &node_p->lock );
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacUpdateContext returned %d", ret);
 
    return ret;
}

/******************************************************************************
 * Function Name : ipacSetRequestParameters
 * Inputs        : request_p - Pointer to request structure to fill in
 *                 needed    - Number of bytes to allow for keystream data
 *                 filp      - file info associated with user's file descriptor
 *                 start_p   - Pointer to start request info from user
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Setup a request structure with enough information to start
 *                 the the request and to return the results to the user once
 *                 the request has completed.
 *
 *****************************************************************************/
static void ipacSetRequestParameters( IpacRequest     *request_p,
                                      u32              needed,
                                      struct file     *filp,
                                      IpacStartIoctl  *start_p )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacContext     *context_p = &node_p->context[start_p->contextId];
    unsigned long    flags;

    PRINTD(DBG_FUNC_CALLS, "ipacSetRequestParameters(%p, %u, %p, %p)",
                            request_p, needed, filp, start_p);

    spin_lock_irqsave( &ipacDevice.hw_lock, flags );

    /* This info has to be returned to the user */ 
    request_p->transactionId = start_p->transactionId;
    request_p->contextId     = start_p->contextId;
    request_p->numberOfPdus  = start_p->numPdus;
    request_p->pduLength     = start_p->pduLength;

    /* This info is needed to submit the request and handle the completion */
    request_p->channel       = node_p - ipacNodes;
    request_p->key_p         = context_p->key_p;
    request_p->ivFixedPart   = context_p->ivFixedPart;
    request_p->reservedBytes = needed;
    request_p->pdusCompleted = 0;
    
    /* Track use of each cipher so we know when it's safe to clean them up */
    context_p->cipherActiveRequests += start_p->numPdus;
    
    spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );

    PRINTD(DBG_FUNC_CALLS, "ipacSetRequestParameters out");
}


/******************************************************************************
 * Function Name : ipacStartContiguousCipherRequests
 * Inputs        : request_p   - Pointer to request structure to fill in
 *                 pduBuffer   - Physical address of block of dest buffers
 *                 contextAddr - pointer to first context area for this block
 *                 numPdus     - number of PDUs to add
 *                 index       - context index of the first PDU
 *                 counts_p    - pointer to array of count values for the IV
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Write the keys and IVs to a block of numPdu contiguous
 *                 contexts.  Submit these requests to the cipher engine.
 *
 *                 This function must always be called with a lock on the HW.
 *
 *****************************************************************************/
static void ipacStartContiguousCipherRequests( IpacRequest  *request_p,
                                               u32           pduBuffer,
                                               u32 __iomem  *contextAddr,
                                               u32           numPdus,
                                               u32           index,
                                               u32          *counts_p )
{
    u32            pduLength = request_p->pduLength;
    u32            pdu;
    u32            key0;
    u32            key1;
    u32            key2;
    u32            key3;
    u32            iv1;
    u32            ddtAddrValue;
    u32            keySizeValue;
    u32            controlValue;
    u32 __iomem   *dstAddr_p;
    u32 __iomem   *keySize_p;
    u32 __iomem   *control_p;
    IpacDdt       *ddt_p;
    IpacRequest  **req_pp;
        
    PRINTD(DBG_FUNC_CALLS, "ipacStartContiguousCipherRequests(%p, %x, %p, %d, %d, %p)",
                            request_p, pduBuffer, contextAddr, numPdus, index, counts_p);
                            
    /* Write the key and iv to each of the contexts in turn.  We
     * should only have to load the key and iv1 into registers once.
     * In this loop, we can also write the request_p numPdus
     * times to the circular buffer of active requests.
     */
    key0 = request_p->key_p[0];
    key1 = request_p->key_p[1];
    key2 = request_p->key_p[2];
    key3 = request_p->key_p[3];
    iv1  = request_p->ivFixedPart;
    
    req_pp = &ipacDevice.activeRequests[index];
    
    for (pdu=0; pdu<numPdus; ++pdu)
    {
        u32   iv0 = *counts_p++;

        PRINTD( DBG_IO, "Key = %08X %08X %08X %08X, IV = %08X %08X at %p\n", key0, key1, key2, key3, iv0, iv1, contextAddr);
        
        iowrite32( key0, contextAddr++ );
        iowrite32( key1, contextAddr++ );
        iowrite32( key2, contextAddr++ );
        iowrite32( key3, contextAddr++ );
        iowrite32( iv0,  contextAddr++ );
        iowrite32( iv1,  contextAddr );
        contextAddr += 11;
        
        *req_pp++ = request_p;
    }

    /* Now we can write the requests to the cipher engine.  We update
     * the Data Description Table for each request and then write its
     * info to the engine, followed by the key size and the trigger.
     * Because these registers contain the context index, we must
     * update them every time.
     */
    ddt_p = &ipacDevice.dst_ddt_buf[index].ddt;
    
    ddtAddrValue = ipacDevice.dst_ddt_buf_phys + index*sizeof(IpacDdt);
    dstAddr_p    = ipacDevice.crypto_regs + SPA_DST_PTR_REG_OFFSET;
    
    keySizeValue = IPAC_KEY_SZ_VALUE(index);
    keySize_p    = ipacDevice.crypto_regs + SPA_KEY_SZ_REG_OFFSET;
    
    controlValue = IPAC_CTRL_VALUE(index);
    control_p    = ipacDevice.crypto_regs + SPA_CTRL_REG_OFFSET(IPAC_CIPHER_KEY_VERSION);

    for (pdu=0; pdu<numPdus; ++pdu)
    {
        PRINTD( DBG_IO, "Key = %08x %u at %p\n", pduBuffer, pduLength, ddt_p);
        
        ddt_p->p0   = pduBuffer;
        ddt_p->len0 = pduLength;
        ddt_p++;
        pduBuffer += pduLength;
        
        PRINTD( DBG_IO, "%08X to %p\n", ddtAddrValue, dstAddr_p);
        iowrite32( ddtAddrValue, dstAddr_p );
        ddtAddrValue += sizeof(IpacDdt);
        
        PRINTD( DBG_IO, "%08X to %p\n", keySizeValue, keySize_p);
        iowrite32( keySizeValue, keySize_p );
        keySizeValue += IPAC_KEY_SZ_INCREMENT;
        
        wmb();

        /* Setup the control register to start the cipher request. */
        PRINTD( DBG_IO, "%08X to %p\n", controlValue, control_p);
        iowrite32( controlValue, control_p );
        controlValue += IPAC_CTRL_VALUE_INCREMENT;
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacStartContiguousCipherRequests out");
}

/******************************************************************************
 * Function Name : ipacStartCipherRequest
 * Inputs        : request_p   - Pointer to request structure to fill in
 *                 counts_p    - pointer to array of count values for the IV
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Submit all the PDUs for a single request to the cipher
 *                 engine.  This request may need to wrap around in the cipher
 *                 engine's contexts (and hence the DDT blocks).  If it does
 *                 we split the request into two contiguous requests to try
 *                 to minimise the overhead or wrapping in the normal case.
 *
 *                 This function must always be called with a lock on the HW
 *                 via the IRQ spinlock.
 *
 *****************************************************************************/
static void ipacStartCipherRequest( IpacRequest  *request_p,
                                    u32          *counts_p,
                                    u32           numPdus,
                                    u32           alreadyDone )
{
    IpacDeviceNode  *node_p      = &ipacNodes[request_p->channel];
    u32              pduBuffer   = (u32)(request_p + 1) - (u32)node_p->dmaOffset + (alreadyDone * request_p->pduLength);
    u32 __iomem     *contextAddr = ipacDevice.cipher_ctx_base + (ipacDevice.activeRequestHead * IPAC_L2_CIPHER_CONTEXT_SIZE);
    
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipherRequest(%p, %p, %d, %d)",
                            request_p, counts_p, numPdus, alreadyDone);
                            
    request_p->state = IPAC_REQUEST_CIPHERING;

    /* All these PDUs share a common length, so we can set it once for all. */
    ipacRegWrite( SPA_PROC_LEN_REG_OFFSET(IPAC_CIPHER_KEY_VERSION), request_p->pduLength );

    if (ipacDevice.activeRequestHead + numPdus <= IPAC_NUM_CIPHER_ENGINE_REQUESTS)
    {
        ipacStartContiguousCipherRequests(request_p, pduBuffer, contextAddr, numPdus, ipacDevice.activeRequestHead, counts_p);
    
        ipacDevice.activeRequestHead += numPdus;
        ipacDevice.activeRequestHead &= ~IPAC_NUM_CIPHER_ENGINE_REQUESTS;
    }
    else
    {
        u32 n1 = IPAC_NUM_CIPHER_ENGINE_REQUESTS - ipacDevice.activeRequestHead;
        u32 n2 = numPdus - n1;
        
        ipacStartContiguousCipherRequests(request_p, pduBuffer, contextAddr, n1, ipacDevice.activeRequestHead, counts_p);
        
        ipacDevice.activeRequestHead  = 0;
        counts_p              += n1;
        pduBuffer             += n1 * request_p->pduLength;
        contextAddr            = ipacDevice.cipher_ctx_base;
        
        ipacStartContiguousCipherRequests(request_p, pduBuffer, contextAddr, n2, ipacDevice.activeRequestHead, counts_p);

        ipacDevice.activeRequestHead = n2;
    }

    ipacDevice.numActiveRequests += numPdus;
    PRINTD(DBG_PDU_COUNT, "Incremented active requests to %d", ipacDevice.numActiveRequests);
    
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipherRequest out");
}

/******************************************************************************
 * Function Name : ipacSubmitRequest
 * Inputs        : node_p    - Pointer to device node
 *                 request_p - Pointer to request header in memory buffer
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Takes the information for a cipher request from the
 *                 memory pool and submits it to the cipher engine.
 *
 *****************************************************************************/
static int ipacSubmitRequest( IpacDeviceNode  *node_p,
                              IpacRequest     *request_p,
                              IpacStartIoctl  *start_p )
{
    u32            *counts = node_p->tempCounts;
    int            ret = 0;
    unsigned long  flags;
    u32            pdu;
    u32            numPdus;
    
    PRINTD(DBG_FUNC_CALLS, "ipacSubmitRequest(%p, %p, %p)", node_p, request_p, start_p);
 
    /* Before submitting more, take the opportunity to handle completed PDUs,
     * to try to ensure that there will be space for more requests.
     */
    ipacProcessCompletedPdus();

    /* Is there enough space for these requests */
    if (request_p == NULL)
    {
        PRINTD(DBG_ERROR, "Invalid (null) request_p");
        ret = -EINVAL;
        goto out;
    }

    numPdus = request_p->numberOfPdus;
    
    if (numPdus > IPAC_MAX_NUM_PDUS)
    {
        PRINTD(DBG_NOTICE, "Number of PDUs in a single request (%d) is > %d",
                            numPdus, IPAC_MAX_NUM_PDUS);
        ret = -EINVAL;
        goto out;
    }
    
    /* Get the counts (outside the lock) */
    if (copy_from_user(counts, start_p->counts, numPdus*sizeof(u32)))
    {
        PRINTD(DBG_NOTICE, "Unable to copy counts[%d] from user space at %p",
                            numPdus, start_p->counts);
        ret = -EINVAL;
        goto out;
    }

    spin_lock_irqsave( &ipacDevice.hw_lock, flags );
        
    if (ipacDevice.numQueuedRequests + 1 > IPAC_MAX_NUM_QUEUED_REQUESTS)
    {
        PRINTD(DBG_ERROR, "Invalid request.  Too many queued requests (%d).", ipacDevice.numQueuedRequests+1);
        ret = -EINVAL;
        goto unlock;
    }
    
    if (ipacDevice.numQueuedCounts + numPdus > IPAC_MAX_NUM_QUEUED_COUNTS)
    {
        PRINTD(DBG_ERROR, "Invalid request.  Would have too many queued PDUS (%d).", ipacDevice.numQueuedCounts+numPdus);
        ret = -EINVAL;
        goto unlock;
    }
    
    /* Queue these requests. Note that we always put requests on the queue.  Requests may
     * be much larger that the cipher engine is able to process in one go, and the queue 
     * provides a way to break these into smaller chunks.
     */
    for (pdu=0; pdu<numPdus; ++pdu)
    {
        ipacDevice.queuedCounts[ipacDevice.queuedCountHead++] = htonl(counts[pdu]);
        ipacDevice.queuedCountHead &= ~IPAC_MAX_NUM_QUEUED_COUNTS;
           
        PRINTD(DBG_PDU_QUEUE, "Queueing count %d", counts[pdu]);
    }
    ipacDevice.numQueuedCounts += numPdus;

    ipacDevice.queuedRequestsDone[ipacDevice.queuedRequestHead] = 0;
    ipacDevice.queuedRequests[ipacDevice.queuedRequestHead++] = request_p;
    ipacDevice.queuedRequestHead &= ~IPAC_MAX_NUM_QUEUED_REQUESTS;
        
    ipacDevice.numQueuedRequests++;
    
unlock:
    spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );

out:

    /* Take the chance to submit any PDUs that are queued */
    ipacProcessQueuedPdus();

    PRINTD(DBG_BUFFER, "After submit requestpool=%p, end=%p, first=%p, last=%p",
                        node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                        node_p->lastRequest_p);
    PRINTD(DBG_FUNC_CALLS, "ipacSubmitRequest returned %d", ret);
 
    return ret;
}

/******************************************************************************
 * Function Name : ipacProcessQueuedPdus
 * Inputs        : None
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Check to see if there are any requests that could be taken
 *                 from the backlog of queued requests and submitted to the
 *                 cipher engine.
 *
 *****************************************************************************/
static void ipacProcessQueuedPdus( void )
{
    unsigned long  flags;
    
    PRINTD(DBG_FUNC_CALLS, "ipacProcessQueuedPdus()");

    /* Get exclusive access to the queued state. */
    spin_lock_irqsave( &ipacDevice.hw_lock, flags );
        
    while (ipacDevice.numQueuedRequests > 0)
    {
        IpacRequest     *request_p     = ipacDevice.queuedRequests[ipacDevice.queuedRequestTail];
        u32              pdusDone      = ipacDevice.queuedRequestsDone[ipacDevice.queuedRequestTail];
        u32              pdusToDo      = request_p->numberOfPdus - pdusDone;
        u32              cipherSpace   = IPAC_NUM_CIPHER_ENGINE_REQUESTS - ipacDevice.numActiveRequests;
        u32              counts[IPAC_NUM_CIPHER_ENGINE_REQUESTS];
        int              i;

        if (cipherSpace == 0)
        {
            /* No space for even a single request */
            goto unlock;
        }

        /* Don't submit more than we're allowed to */
        if (pdusToDo > cipherSpace)
        {
            pdusToDo = cipherSpace;
        }

        /* Submit that many PDUs */
        for (i=0; i<pdusToDo; ++i)
        {
            counts[i] = ipacDevice.queuedCounts[ipacDevice.queuedCountTail++];
            ipacDevice.queuedCountTail &= ~IPAC_MAX_NUM_QUEUED_COUNTS;
            PRINTD(DBG_PDU_QUEUE, "Retrieved count %d", counts[i]);
        }
        ipacDevice.numQueuedCounts -= pdusToDo;
        
        ipacStartCipherRequest(request_p, counts, pdusToDo, pdusDone);

        /* If we've submitted all the PDUs for this request then we can move on */
        ipacDevice.queuedRequestsDone[ipacDevice.queuedRequestTail] += pdusToDo;

        if (ipacDevice.queuedRequestsDone[ipacDevice.queuedRequestTail] >= request_p->numberOfPdus)
        {
            ipacDevice.queuedRequestTail++;
            ipacDevice.queuedRequestTail &= ~IPAC_MAX_NUM_QUEUED_REQUESTS;
        
            ipacDevice.numQueuedRequests--;
        }
    }

unlock:
    spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );
        
    PRINTD(DBG_FUNC_CALLS, "ipacProcessQueuedPdus out");
}

/******************************************************************************
 * Function Name : ipacCipherComplete
 * Inputs        : req_p - Pointer to the cipher request that was submitted
 *                 err   - Status of the cipher request
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Called when the cipher engine has completed a cipher
 *                 request.  The request in the memory pool is updated.
 *
 *                 This function should always be called with a lock on the HW
 *                 via the IRQ spinlock
 *
 *****************************************************************************/
static void ipacCipherComplete( IpacRequest  *request_p,
                                int           err )
{
    IpacDeviceNode  *node_p = &ipacNodes[request_p->channel];
    
    PRINTD(DBG_FUNC_CALLS, "ipacCipherComplete(%p, %d)", request_p, err);
 
    if ( !err )
    {
        u32          *key_p     = request_p->key_p;
        int           contextId = request_p->contextId;
        IpacContext  *context_p = &(node_p->context[contextId]);
        
        if (key_p == context_p->key_p)
        {
            PRINTD(DBG_CIPHER_REQ, "Completed cipher request for current cipher %p", key_p);
            
            context_p->cipherActiveRequests--;
        }
        else if (key_p == context_p->oldKey_p)
        {
            PRINTD(DBG_CIPHER_REQ, "Completed cipher request for old cipher %p", key_p);
            
            context_p->oldCipherActiveRequests--;
            
            if (context_p->oldCipherActiveRequests == 0)
            {
                /* This request will not be used any more */
                PRINTD(DBG_CIPHER, "Freeing old cipher %p; no old clients left", key_p);
                context_p->oldKey_p = NULL;
            }
        }
        
        request_p->pdusCompleted++;
        PRINTD(DBG_PDU_COUNT, "Completed %d pdus of %d", request_p->pdusCompleted, request_p->numberOfPdus);
        if (request_p->pdusCompleted == request_p->numberOfPdus)
        {
            /* Entire request has been completed and can now be returned */
            request_p->state = IPAC_REQUEST_COMPLETED;
        }
    }
    else
    {
        PRINTD(DBG_NOTICE, "encryption on keystream channel %lu failed "
                "with code %d", request_p->channel, err );
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacCipherComplete out");
}


/******************************************************************************
 * Function Name : ipacProcessCompletedPdus
 * Inputs        : None
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Check to see if any cipher requests have completed.  If so
 *                 take them from the cipher engine and handle their result.
 *
 *****************************************************************************/
static void ipacProcessCompletedPdus( void )
{
    unsigned long  flags;
    
    PRINTD(DBG_FUNC_CALLS, "ipacProcessCompletedPdus()");

    spin_lock_irqsave( &ipacDevice.hw_lock, flags );
        
    while ( ! ( ipacRegRead( SPA_FIFO_STAT_REG_OFFSET ) & SPA_FIFO_STAT_EMPTY ) )
    {
        int  result;
        
        /* POP the status register. */
        ipacRegWrite( SPA_STAT_POP_REG_OFFSET, ~0 );
        wmb();

        /* Read the status of the operation. */
        result = ( ipacRegRead( SPA_STATUS_REG_OFFSET ) & SPA_STATUS_RES_CODE_MASK ) >> SPA_STATUS_RES_CODE_OFFSET;

        /* Convert the SPAcc error status into the standard POSIX error
         * codes. */
        switch ( result )
        {
            case SPA_STATUS_OK:
                result = 0;
                break;

            case SPA_STATUS_ICV_FAIL:
                result = -EBADMSG;
                break;

            case SPA_STATUS_MEMORY_ERROR:
                PRINTD( DBG_WARN, "memory error triggered" );
                result = -EFAULT;
                break;

            case SPA_STATUS_BLOCK_ERROR:
                PRINTD( DBG_WARN, "block error triggered" );
                result = -EIO;
                break;

            default:
                BUG();
        }

        /* Check that we have a request corresponding to this */
        if ( ipacDevice.numActiveRequests > 0 )
        {
            IpacRequest *request_p = ipacDevice.activeRequests[ipacDevice.activeRequestTail++];
            ipacDevice.activeRequestTail &= ~IPAC_NUM_CIPHER_ENGINE_REQUESTS;
            
            ipacDevice.numActiveRequests--;
            PRINTD(DBG_PDU_COUNT, "Decrementdc active requests to %d", ipacDevice.numActiveRequests);
            
            ipacCipherComplete(request_p, result);
        }
        else
        {
            BUG();
        }
    }
    
    spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );
    
    PRINTD(DBG_FUNC_CALLS, "ipacProcessCompletedPdus out");
}

/******************************************************************************
 * Function Name : ipacCleanupCompletedPdus
 * Inputs        : None
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Check to see if there are any cipher requests that have
 *                 completed but not been handled.  If so take them from the
 *                 cipher engine and throw their result away.  This is only
 *                 used during driver start up to ensure the HW state is known.
 *
 *****************************************************************************/
static void ipacCleanupCompletedPdus( void )
{
    PRINTD(DBG_FUNC_CALLS, "ipacCleanupCompletedPdus()");
    
    while ( ! ( ipacRegRead( SPA_FIFO_STAT_REG_OFFSET ) & SPA_FIFO_STAT_EMPTY ) )
    {
        int  result;
        
        /* POP the status register. */
        ipacRegWrite( SPA_STAT_POP_REG_OFFSET, ~0 );
        wmb();

        /* Read the status of the operation. */
        result = ( ipacRegRead( SPA_STATUS_REG_OFFSET ) & SPA_STATUS_RES_CODE_MASK ) >> SPA_STATUS_RES_CODE_OFFSET;

    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacCleanupCompletedPdus out");
}

/******************************************************************************
 * Function Name : ipacAlignmentForDMA
 * Inputs        : None
 * Outputs       : -
 * Returns       : -
 *
 * Description   : There's a strict DMA HW requirement that each PDU is in a
 *                 single 64K memory region.  When there was just a single 64K
 *                 buffer that was guaranteed.  If we allow up to 128K, or more,
 *                 then we have to be careful to ensure that no PDU crosses a
 *                 64K boundary.  We enforce this by inserting a dummy SKIP
 *                 request to force the required alignment.
 *
 *                 If we insert a dummy request, we also have to ensure that the
 *                 start of the next request is aligned on a 32 byte cacheline
 *                 boundary.  So the dummy alignment request we introduce must
 *                 be a multiple of 32 bytes.
 *
 *                 The worst case is that we'll have to put the very first PDU
 *                 of the actual request at the start of the next DMA boundary.
 *                 However, we'll normally be able to do much better than that,
 *                 since 5*32=160=2*80.  So moving by 1 or 2 PDUs is likely to
 *                 be all that's needed.
 *
 *****************************************************************************/
static int ipacAlignmentForDma(IpacDeviceNode  *node_p,
                               u8              *request_p,
                               u32              numPdus,
                               u32              pduLength)
{
    int  startOfPdus   = request_p + sizeof(IpacRequest) - node_p->startOfPool_p + node_p->startOfDma_p;
    int  firstDmaBlock = startOfPdus & IPAC_DMA_HW_PAGE_MASK;
    int  lastDmaBlock  = (startOfPdus + numPdus*pduLength - 1) & IPAC_DMA_HW_PAGE_MASK;
    int  align;
    int  pdu;
    int  gap;
        
    if (firstDmaBlock == lastDmaBlock)
    {
        /* All the PDUs lie in a single page, no need for further alignment */
        PRINTD(DBG_DMA_ALIGN, "Single page for request_p=%p (%08x, %08x) and %d x %d byte PDUs",
                               request_p, node_p->startOfDma_p, startOfPdus, numPdus, pduLength);

        return 0;
    }

    if (firstDmaBlock+IPAC_DMA_HW_ALIGNMENT != lastDmaBlock)
    {
        /* The request is more than 64K.  That's not meant to be possible.
         * We're supposed to have a limit of 512 PDUs of 80 bytes each.
         * If that limit is being exceeded then we can't do anything here
         * in the general case.  At least 1 PDU is going to get incorrect
         * cipher data.
         */
        PRINTD(DBG_DMA_ALIGN, "Double page for request_p=%p (%08x, %08x) and %d x %d byte PDUs",
                               request_p, node_p->startOfDma_p, startOfPdus, numPdus, pduLength);

        return 0;
    }

    PRINTD(DBG_DMA_ALIGN, "Have to fix up request_p=%p (%08x, %08x) and %d x %d byte PDUs",
                           request_p, node_p->startOfDma_p, startOfPdus, numPdus, pduLength);

    /* Worst case is the default of having to move the entire request into the
     * start of the next DMA block.
     */
    align = (IPAC_DMA_HW_ALIGNMENT - (startOfPdus & IPAC_DMA_HW_OFFSET_MASK)) & IPAC_DMA_HW_OFFSET_MASK;

    /* Find the pdu closest to the DMA page boundary that starts at or
     * before the DMA page boundary and that could be shifted by a multiple
     * of 32 bytes to lie exactly on the page boundary.
     */
    pdu = align / pduLength;
    gap = align - pdu*pduLength;

    PRINTD(DBG_DMA_ALIGN, "Fix up checking from align=%d pdu=%d gap=%d", align, pdu, gap);

    while (pdu > 0)
    {
        if ((gap % sizeof(IpacRequest)) == 0)
        {
            /* This is the closest PDU that could be aligned with the DMA 64K boundary
             * while retaining the IpacRequest 32 byte alignment.
             */
            align = gap;
            PRINTD(DBG_DMA_ALIGN, "Fixing up with pdu=%d, gap=%d", pdu, gap);
            break;
        }

        pdu--;
        gap += pduLength;
    }

    PRINTD(DBG_DMA_ALIGN, "Forcing alignment of %d for request_p=%p and %d x %d byte PDUs", align, request_p, numPdus, pduLength);

    return align;
}

/******************************************************************************
 * Function Name : ipacAllocateRequestBuffer
 * Inputs        : filp      - file info for user's file descriptor
 *                 start_p   - Pointer to the user's request information
 *                 numPdus   - Number of PDUs to allocate
 *                 pduLength - Size of each PDU in bytes
 * Outputs       : buffer_pp - Pointer to allocated request header
 * Returns       : status 0 => OK
 *
 * Description   : Checks for space for the PDUs.  If the caller has passed
 *                 in a non-NULL buffer_pp then the buffer space will actually
 *                 be reserved and the header of the reserved buffer will be
 *                 returned in buffer_pp.
 *
 *****************************************************************************/
static int ipacAllocateRequestBuffer( struct file      *filp,
                                      IpacStartIoctl   *start_p,
                                      u32               numPdus,
                                      u32               pduLength,
                                      IpacRequest     **buffer_pp )
{
    IpacDeviceNode  *node_p     = filp->private_data;
    IpacRequest     *buffer     = NULL;
    IpacRequest     *lastBuffer = NULL;
    IpacRequest     *wrapBuffer = NULL;
    IpacRequest     *dmaBuffer  = NULL;
    static u32       align      = 0xFFFFFFFF;
    int              needed     = numPdus * pduLength;
    int              ret        = -ENOMEM;
    
    PRINTD(DBG_FUNC_CALLS, "ipacAllocateRequestBuffer(%p, %p, %u, %u, %p)",
                            filp, start_p, numPdus, pduLength, buffer_pp);
    
    spin_lock_bh( &node_p->lock );
    
    if (align == 0xFFFFFFFF)
    {
        /* Determine the minimum alignment requirement for the IpacRequest - 1 */
        align = offsetof(struct {u8 dummy; IpacRequest x; }, x) - 1;
        if (align < 31)
        {
             align = 31;
        }
        PRINTD(DBG_BUFFER, "buffer alignment is %u", align + 1);
    }
    
    /* Round the needed bytes up to keep the headers correctly aligned. */
    needed += align;
    needed |= align;
    needed -= align;

    PRINTD(DBG_BUFFER, "Need %u bytes for %u of %u", needed, numPdus, pduLength);
    
    if (node_p->firstRequest_p > node_p->lastRequest_p)
    {
        /* Buffer has wrapped, so the available space lies between the last and first
         *                |......L         F.....  |
         */
        int  midSpace = node_p->firstRequest_p - node_p->lastRequest_p - 2*sizeof(IpacRequest);
        int  alignDMA = ipacAlignmentForDma(node_p, node_p->lastRequest_p, numPdus, pduLength);
        
        PRINTD(DBG_BUFFER, "MID space is %d bytes", midSpace);

        if (midSpace > (needed + alignDMA))
        {
            PRINTD(DBG_BUFFER, "Can use MID space");
            if (buffer_pp)
            {
                if (alignDMA)
                {
                    dmaBuffer  = (IpacRequest *)node_p->lastRequest_p;
                    node_p->lastRequest_p += alignDMA;
                    memset(dmaBuffer, 0, sizeof(dmaBuffer));
                    dmaBuffer->reservedBytes = alignDMA - sizeof(IpacRequest);
                    dmaBuffer->state = IPAC_REQUEST_SKIP;
                }

                buffer  = (IpacRequest *)node_p->lastRequest_p;
                
                node_p->lastRequest_p += needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Inserting new=%p, last=%p", buffer, lastBuffer);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
        }
/*
        else
        {
            PRINTD(DBG_NOTICE, "MID space not enough, %d asked for, %d available", needed, midSpace);
            PRINTD(DBG_NOTICE, "(%d pdus of %d bytes, DMA align=%d)", numPdus, pduLength, alignDMA);
            PRINTD(DBG_NOTICE, "Buffer start %p, first=%p, last=%p, end=%p",
                       (void*)node_p->startOfPool_p, (void*)node_p->firstRequest_p, (void*)node_p->lastRequest_p, (void*)node_p->endOfPool_p);  
      }
*/
    }
    else
    {
        /* Buffer hasn't wrapped, so there's potentially space both after the last and
         * before the first.
         *           |    F.......L   |
         */
         int  headSpace = node_p->firstRequest_p - node_p->startOfPool_p - 2*sizeof(IpacRequest);
         int  tailSpace = node_p->endOfPool_p    - node_p->lastRequest_p - 2*sizeof(IpacRequest);
         int  alignDMA  = ipacAlignmentForDma(node_p, node_p->lastRequest_p, numPdus, pduLength);
         
         PRINTD(DBG_BUFFER, "HEAD space is %d bytes", headSpace);
         PRINTD(DBG_BUFFER, "TAIL space is %d bytes", tailSpace);

         if (tailSpace >= (needed+alignDMA))
         {
            PRINTD(DBG_BUFFER, "Can use TAIL space");
            if (buffer_pp)
            {
                if (alignDMA)
                {
                    dmaBuffer  = (IpacRequest *)node_p->lastRequest_p;
                    node_p->lastRequest_p += alignDMA;
                    memset(dmaBuffer, 0, sizeof(dmaBuffer));
                    dmaBuffer->reservedBytes = alignDMA - sizeof(IpacRequest);
                    dmaBuffer->state = IPAC_REQUEST_SKIP;
                }

                buffer  = (IpacRequest *)node_p->lastRequest_p;
                node_p->lastRequest_p += needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Appending new=%p, last=%p", buffer, lastBuffer);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
         }
         else if (headSpace >= needed)
         {
            PRINTD(DBG_BUFFER, "Can use HEAD space");
            if (buffer_pp)
            {
                wrapBuffer = (IpacRequest *)node_p->lastRequest_p;
                buffer = (IpacRequest *)node_p->startOfPool_p;
                node_p->lastRequest_p = node_p->startOfPool_p + needed + sizeof(IpacRequest);
                lastBuffer = (IpacRequest *)node_p->lastRequest_p;
                
                PRINTD(DBG_BUFFER, "Wrapping wrap=%p, new=%p, last=%p", wrapBuffer, buffer, lastBuffer);
                ipacSetRequestParameters(buffer, needed, filp, start_p);
                memset(lastBuffer, 0, sizeof(lastBuffer));
                
                lastBuffer->state = IPAC_REQUEST_FINAL;
                wrapBuffer->state = IPAC_REQUEST_WRAPPED;
                buffer->state = IPAC_REQUEST_WAITING;

                *buffer_pp = buffer;
            }
            
            ret = 0;
         }
/*
        else
        {
            PRINTD(DBG_NOTICE, "HEAD and TAIL space not enough, %d asked for, %d head available, %d tail available", needed, headSpace, tailSpace);
            PRINTD(DBG_NOTICE, "(%d pdus of %d bytes, DMA align=%d)", numPdus, pduLength, alignDMA);
            PRINTD(DBG_NOTICE, "Buffer start %p, first=%p, last=%p, end=%p",
                       (void*)node_p->startOfPool_p, (void*)node_p->firstRequest_p, (void*)node_p->lastRequest_p, (void*)node_p->endOfPool_p);
        }
*/
    }
    
    PRINTD(DBG_BUFFER, "After alloc request pool=%p, end=%p, first=%p, last=%p",
                        node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                        node_p->lastRequest_p);
    
    spin_unlock_bh( &node_p->lock );

    if (dmaBuffer)
    {
        ipacFlushRequestInfo(dmaBuffer);
    }
    if (lastBuffer)
    {
        ipacFlushRequestInfo(lastBuffer);
    }
    if (wrapBuffer)
    {
        ipacFlushRequestInfo(wrapBuffer);
    }
    if (buffer)
    {
        ipacFlushRequestInfo(buffer);
    }

    PRINTD(DBG_FUNC_CALLS, "ipacAllocateRequestBuffer returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacCheckSpace
 * Inputs        : filp             - Info associated with user's file desc.
 *                 maxSpaceRequired - Number of bytes needed.
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Ioctl handler function for determining if there is space
 *                 available for a request.
 *
 *****************************************************************************/
static int ipacCheckSpace( struct file  *filp,
                           u32           maxSpaceRequired )
{
    IpacDeviceNode  *node_p = filp->private_data;
    int              ret    = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckSpace(%p, %u)", filp, maxSpaceRequired);
    
    if (IPAC_MAX_NUM_BUFFERS_ALLOWED && (node_p->requestsPending >= IPAC_MAX_NUM_BUFFERS_ALLOWED))
    {
        PRINTD(DBG_NOTICE, "More than the maximum number of requests would be active");
        ret = -EBUSY;
        goto out;
    }
    
    ret = ipacAllocateRequestBuffer(filp, NULL, 1, maxSpaceRequired, NULL);
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckSpace returned %d", ret);

out:    
    return ret;
}

/******************************************************************************
 * Function Name : ipacStartCipher
 * Inputs        : filp    - Info associated with user's file descriptor.
 *                 start_p - Pointer to the user data passed in the ioctl
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Validate a user request for generating cipher data.  If
 *                 it's OK then add the request to the buffer and mark it
 *                 waiting.  If the request can then be submitted to the
 *                 cipher engine then do that too.
 *
 *****************************************************************************/
static int ipacStartCipher( struct file     *filp,
                            IpacStartIoctl  *start_p )
{
    IpacDeviceNode  *node_p      = filp->private_data;
    IpacRequest     *request_p   = NULL;
    int              ret         = -ENOMEM;
   
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipher(%p, %p)", filp, start_p);
    
    
    /* Ensure we have a valid context */
    if (!node_p->context[start_p->contextId].key_p)
    {
        PRINTD(DBG_NOTICE, "Context is not valid");
        ret = -EINVAL;
        goto out;
    }
   
    /* Ensure the pduLength is a multiple of 4 */
    if (start_p->pduLength % 4)
    {
        PRINTD(DBG_NOTICE, "PDU is not a mutiple of 4 bytes");
        ret = -EINVAL;
        goto out;
    }

    if (start_p->numPdus > IPAC_MAX_NUM_PDUS)
    {
        PRINTD(DBG_NOTICE, "Number of PDUs in a single request (%d) is > %d",
                            start_p->numPdus, IPAC_MAX_NUM_PDUS);
        ret = -EINVAL;
        goto out;
    }
    
    if (IPAC_MAX_NUM_BUFFERS_ALLOWED && (node_p->requestsPending >= IPAC_MAX_NUM_BUFFERS_ALLOWED))
    {
        PRINTD(DBG_NOTICE, "More than the maximum number of requests would be active");
        ret = -EBUSY;
        goto out;
    }
    
    PRINTD(DBG_PDU_COUNT, "Requesting %u pdus of %u bytes", start_p->numPdus, start_p->pduLength);
    ret = ipacAllocateRequestBuffer(filp, start_p, start_p->numPdus, start_p->pduLength, &request_p);
    if (ret)
    {
        PRINTD(DBG_NOTICE, "Request failed %d", ret);
        goto out;
    }
    
    node_p->requestsPending++;
    
    ret = ipacSubmitRequest(node_p, request_p, start_p);
    if (ret != 0)
    {
        unsigned long  flags;

        /* If the submit failed after we had already allocated buffer space, then we should
         * make sure that this request isn't returned to the user app and that these PDUs
         * do not keep the cipher context active.
         */
        spin_lock_irqsave( &ipacDevice.hw_lock, flags );

        node_p->context[start_p->contextId].cipherActiveRequests -= start_p->numPdus;
        request_p->pdusCompleted = request_p->numberOfPdus;
        request_p->state = IPAC_REQUEST_SKIP;

        spin_unlock_irqrestore( &ipacDevice.hw_lock, flags );
    }

out:
    
    PRINTD(DBG_FUNC_CALLS, "ipacStartCipher returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipaCheckCryptoReady
 * Inputs        : filp - Info associated with user's file descriptor.
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Check whether there are any requests that have finished
 *                 that can be returned to the user.
 *
 *****************************************************************************/
static int ipacCheckCryptoReady( struct file  *filp )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacRequest     *request_p = NULL;
    IpacRequest     *flush_p   = NULL;
    int              ret       = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacCheckCryptoReady(%p)", filp);
    
    /* Check if the HW engine has completed any more. */
    ipacProcessCompletedPdus();
    
    /* Also take the chance to submit any PDUs that are already queued */
    ipacProcessQueuedPdus();

    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    PRINTD(DBG_BUFFER, "CheckCryptoReady firstRequest_p is %p(%d)",
                        node_p->firstRequest_p, request_p->state);
 
    while ((request_p->state == IPAC_REQUEST_SKIP) || (request_p->state == IPAC_REQUEST_WRAPPED))
    {
        if (request_p->state == IPAC_REQUEST_WRAPPED)
        {
            PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
            flush_p = request_p;
            node_p->firstRequest_p = node_p->startOfPool_p;
        }
        else
        {
            PRINTD(DBG_BUFFER, "Skipping buffer that faulted.");
            request_p->state = IPAC_REQUEST_UNUSED;
            flush_p = request_p;
            node_p->firstRequest_p += request_p->reservedBytes + sizeof(IpacRequest);
            node_p->requestsPending--;
        }
        
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if (request_p->state == IPAC_REQUEST_COMPLETED)
    {
        PRINTD(DBG_BUFFER, "Marking returned buffer at %p", request_p);
        request_p->state = IPAC_REQUEST_RETURNED;

        ret = node_p->firstRequest_p - node_p->startOfPool_p;
    }
    else if (request_p->state == IPAC_REQUEST_RETURNED)
    {
        PRINTD(DBG_BUFFER, "Buffer is still marked as returned at %p", request_p);

        ret = node_p->firstRequest_p - node_p->startOfPool_p;
    }
    else
    {
        ret = -EBUSY;
    }
    
    spin_unlock_bh( &node_p->lock );

    if (flush_p)
    {
        ipacFlushRequestInfo(flush_p);
    }
    
    ipacFlushRequestInfo(request_p);
    
    PRINTD(DBG_FUNC_CALLS, "ipacCheckCryptoReady returned %d", ret);

    return ret;    
}

/******************************************************************************
 * Function Name : ipacRead
 * Inputs        : filp    - Info associated with user's file descriptor
 *                 buf     - User buffer to fill
 *                 buf_len - Max number of bytes to return
 *                 offset  - file offset (not used)
 * Outputs       : -
 * Returns       : number of bytes returned in buf or -ve error code
 *
 * Description   : This provides an alternative interface to the mmap().
 *                 Using this it is possible to read the cipher data back
 *                 with a read() system call.  This function was added for
 *                 initial testing and is not intended for use under normal
 *                 circumstances.
 *
 *****************************************************************************/
static ssize_t ipacRead( struct file  *filp,
                         char __user  *buf,
                         size_t        buf_len,
                         loff_t       *offset )
{
    IpacDeviceNode  *node_p    = filp->private_data;
    IpacRequest     *request_p = NULL;
    int              ret       = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacRead(%p, %p, %d, %p)", filp, buf, buf_len, offset);
    
    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    PRINTD(DBG_BUFFER, "Read firstRequest_p is %p(%d)", node_p->firstRequest_p, request_p->state);
 
    if (request_p->state == IPAC_REQUEST_WRAPPED)
    {
        PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
        node_p->firstRequest_p = node_p->startOfPool_p;
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if ((request_p->state == IPAC_REQUEST_COMPLETED) || (request_p->state == IPAC_REQUEST_RETURNED))
    {
        size_t  bytes_to_copy = min( buf_len, (sizeof(IpacRequest) + request_p->numberOfPdus*request_p->pduLength) );

        /* If an attempt at reading fails, the user space app is free to either try again,
         * or discard the data by unloading it and continuing.
         */
        PRINTD(DBG_BUFFER, "Marking returned buffer at %p", request_p);
        request_p->state = IPAC_REQUEST_RETURNED;
        
        spin_unlock_bh( &node_p->lock );
    
        ret = copy_to_user( buf, request_p, bytes_to_copy );
        if ( ret )
        {
            PRINTD(DBG_NOTICE, "Unable to copy request_p to user space at %p", buf);
            ret = -EFAULT;
        }
        else
        {
            ipacFlushRequestInfo(request_p);
    
            ret = bytes_to_copy;
        }
    }
    else
    {
        spin_unlock_bh( &node_p->lock );
        ret = -EBUSY;
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacRead returned %d", ret);
    
    return ret;
}


/******************************************************************************
 * Function Name : ipacBufferUnload
 * Inputs        : filp - Info associated with user's file descriptor
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : This function is called (via an ioctl) when the user has
 *                 finished with the cipher data in a request buffer.  It
 *                 allows the driver to reclaim the buffer space for further
 *                 requests.
 *
 *****************************************************************************/
static int ipacBufferUnload( struct file  *filp )
{
    IpacDeviceNode  *node_p          = filp->private_data;
    IpacRequest     *request_p       = NULL;
    IpacRequest     *flushPreWrap_p  = NULL;
    IpacRequest     *flushResult_p   = NULL;
    IpacRequest     *flushPostWrap_p = NULL;
    int              ret             = 0;
   
    PRINTD(DBG_FUNC_CALLS, "ipacBufferUnload(%p)", filp);
    
    PRINTD(DBG_BUFFER, "BufferUnload firstRequest_p is %p", node_p->firstRequest_p);
 
    spin_lock_bh( &node_p->lock );
    
    request_p = (IpacRequest*)node_p->firstRequest_p;
    
    if (request_p->state == IPAC_REQUEST_WRAPPED)
    {
        PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
        request_p->state = IPAC_REQUEST_UNUSED;
        flushPreWrap_p = request_p;
        node_p->firstRequest_p = node_p->startOfPool_p;
        request_p = (IpacRequest*)node_p->firstRequest_p;
    }
    
    if (request_p->state != IPAC_REQUEST_RETURNED)
    {
        spin_unlock_bh( &node_p->lock );
        PRINTD(DBG_NOTICE, "Buffer being unloaded isn't completed");
        ret = -EINVAL;
    }
    else
    {
#ifdef ALLOW_CACHED_USER_READS 
        u8   *start_p = ((u8*)request_p - node_p->startOfPool_p) + node_p->vm_start;
        u8   *end_p   = start_p + request_p->reservedBytes + sizeof(IpacRequest);
        u8   *page_p = start_p;
        u8    value;
#endif

        PRINTD(DBG_BUFFER, "Moving past %d bytes of space", request_p->reservedBytes);
    
        flushResult_p = request_p;
        
        request_p->state = IPAC_REQUEST_UNUSED;
        node_p->firstRequest_p += request_p->reservedBytes + sizeof(IpacRequest);
    
        request_p = (IpacRequest*)node_p->firstRequest_p;
    
        node_p->requestsPending--;

        if (request_p->state == IPAC_REQUEST_WRAPPED)
        {
            PRINTD(DBG_BUFFER, "Reached end of the buffer.  Wrapping to start of buffer");
            request_p->state = IPAC_REQUEST_UNUSED;
            flushPostWrap_p = request_p;
            node_p->firstRequest_p = node_p->startOfPool_p;
        }
        
        spin_unlock_bh( &node_p->lock );

#ifdef ALLOW_CACHED_USER_READS
        /* First ensure that the user memory is actually paged for reading */
        while ((page_p >= start_p) & (page_p < end_p))
        {
            get_user(value, page_p);
            page_p += 4096;
            page_p = (u8*)(0xFFFFF000 & (u32)page_p);
        }
        
        ipacInvalidateCipherData(start_p, end_p-1);
#endif
        
        ipacFlushRequestInfo(flushResult_p);
        
        if (flushPostWrap_p)
        {
            ipacFlushRequestInfo(flushPostWrap_p);
        }
        
        PRINTD(DBG_BUFFER, "After buffer unload pool=%p, end=%p, first=%p, last=%p",
                            node_p->startOfPool_p, node_p->endOfPool_p, node_p->firstRequest_p,
                            node_p->lastRequest_p);
    }

    if (flushPreWrap_p)
    {    
        ipacFlushRequestInfo(flushPreWrap_p);
    }
    
    PRINTD(DBG_FUNC_CALLS, "ipacBufferUnload returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacIoctl
 * Inputs        : inode - inode associated with the dev node
 *                 filp  - info associated with the user's file descriptor
 *                 cmd   - the ioctl cmd
 *                 arg   - the arg associated with the cmd
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : This is the ioctl handler for the driver.  It passes any
 *                 recognised commands to specific handler functions.
 *
 *****************************************************************************/
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
static long ipacIoctl(
#else                
static int ipacIoctl( struct inode   *inode,
#endif
                      struct file    *filp,
                      unsigned int    cmd,
                      unsigned long   arg )
{
    IpacStartIoctl    startIoctl;
    IpacContextIoctl  contextIoctl;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
    long              ret          = 0;
#else
    int               ret          = 0;
#endif

    PRINTD(DBG_FUNC_CALLS, "ipacIoctl(%p, %u, %lu)", filp, cmd, arg);
    
    /* don't even decode wrong cmds: better returning  ENOTTY than EFAULT */
    if (_IOC_TYPE(cmd) != IPAC_IOCTL_MAGIC){
        PRINTD(DBG_NOTICE, "Invalid ioctl command 0x%x (arg=%lu)", cmd, arg );
        ret = -ENOTTY;
    }
    else if (_IOC_NR(cmd) > IPAC_IOCTL_MAXNR)
    {
        PRINTD(DBG_NOTICE, "Invalid ioctl command 0x%x (arg=%lu)", cmd, arg );
        ret = -ENOTTY;
    }
    else
    {
        IpacDeviceNode  *node_p       = filp->private_data;

        /* Atomically check whether the file is closing and, if not, indicate
         * to the ipacRelease function that we are busy processing an ioctl
         * request.
         */
        spin_lock_bh( &node_p->lock );
        if (node_p->releasingNode)
        {
            spin_unlock_bh( &node_p->lock );
            ret = -EBUSY;
        }
        else
        {
            node_p->inIoctlCall++;
            spin_unlock_bh( &node_p->lock );

            switch (cmd)
            {
            case IPAC_IOCTL_ALLOC_CONTEXT:
                if (copy_from_user( &contextIoctl, (void*)arg, sizeof(contextIoctl)))
                {
                    PRINTD(DBG_NOTICE, "Unable to copy ioctl data from user");
                    ret = -EINVAL;
                }
                else if (contextIoctl.contextId < 0)
                {
                    ret = ipacAllocateContext(filp, &contextIoctl);
                }
                else
                {
                    ret = ipacUpdateContext(filp, &contextIoctl);
                }
                break;
        
            case IPAC_IOCTL_DEALLOC_CONTEXT:
                ret = ipacDeallocateContext(filp, arg);
                break;
        
            case IPAC_IOCTL_CHECK_SPACE:
                ret = ipacCheckSpace(filp, arg);
                break;
        
            case IPAC_IOCTL_START_CRYPTO:
                if (copy_from_user( &startIoctl, (void*)arg, sizeof(startIoctl)))
                {
                    PRINTD(DBG_NOTICE, "Unable to copy ioctl data from user");
                    ret = -EINVAL;
                }
                else
                {
                    ret = ipacStartCipher(filp, &startIoctl);
                }
                break;
        
            case IPAC_IOCTL_CHECK_CRYPTO_READY:
                ret = ipacCheckCryptoReady(filp);
                break;
        
            case IPAC_IOCTL_BUFFER_UNLOAD:
                ret = ipacBufferUnload(filp);
                break;
        
            default:
                /* redundant, as cmd was checked against MAXNR */
                ret = -ENOTTY;
            }

            /* No longer busy in ioctl */
            spin_lock_bh( &node_p->lock );
            if (node_p->inIoctlCall > 0)
            {
                node_p->inIoctlCall--;
            }
            spin_unlock_bh( &node_p->lock );
        }
    }

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 38)
    PRINTD(DBG_FUNC_CALLS, "ipacIoctl returned %ld", ret);
#else
    PRINTD(DBG_FUNC_CALLS, "ipacIoctl returned %d", ret);
#endif
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacVmaOpen
 * Inputs        : vma - Pointer to the vma associated with the mmap
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Dummy function
 *
 *****************************************************************************/
static void ipacVmaOpen( struct vm_area_struct *vma )
{
    PRINTD(DBG_FUNC_CALLS, "ipacVmaOpen(%p)", vma );
    PRINTD(DBG_FUNC_CALLS, "ipacVmaOpen out");
}

/******************************************************************************
 * Function Name : ipacVmaClose
 * Inputs        : vma - Pointer to the vma associated with the mmap
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Dummy function
 *
 *****************************************************************************/
static void ipacVmaClose( struct vm_area_struct  *vma )
{
    PRINTD(DBG_FUNC_CALLS, "ipacVmaClose(%p)", vma );
    PRINTD(DBG_FUNC_CALLS, "ipacVmaClose out");
}

/******************************************************************************
 * Function Name : ipacVmaFault 
 * Inputs        : vma - Pointer to the vma associated with the mmap
 *                 vmf - Pointer to a structure for passing page fault info.
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : This is the mmap page fault handler.  As the user space
 *                 accesses pages in the mapped area, this is called to map
 *                 the user space to the kernel pages used for the buffer.
 *
 *****************************************************************************/
static int ipacVmaFault( struct vm_area_struct  *vma,
                         struct vm_fault        *vmf )
{
    IpacDeviceNode     *node_p  = vma->vm_private_data;
    struct page        *page    = NULL;
    u8                 *offset  = NULL;
    int                 ret     = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacVmaFault(%p, %p)", vma, vmf);
    
    PRINTD(DBG_MMAP, "Offsets %p %lx %p.", vmf->virtual_address, vma->vm_start, node_p->startOfPool_p );

    offset = ((unsigned long)vmf->virtual_address - vma->vm_start) + node_p->startOfPool_p;
    if (offset >= node_p->endOfPool_p)
    {
        PRINTD(DBG_NOTICE, "Address %p is beyond end of buffer %p.", offset, node_p->endOfPool_p );
        ret = VM_FAULT_SIGBUS;
    }
    else
    {
        PRINTD(DBG_MMAP, "Offset is %p.", offset );
        
        page = virt_to_page(offset);
        PRINTD(DBG_MMAP, "Page is %p.", page );
        get_page(page);
        vmf->page = page;
    }

    PRINTD(DBG_FUNC_CALLS, "ipacVmaFault returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacMmap
 * Inputs        : filp - Info associated with the user's file descriptor.
 *                 vma  - vma structure describing mapped area.
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Defines a non-cached mapping for the mmap()ed area requested
 *                 by the user.
 *
 *****************************************************************************/
static int ipacMmap( struct file            *filp,
                     struct vm_area_struct  *vma )
{
    IpacDeviceNode  *node_p = filp->private_data;
    int              ret    = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacMmap(%p, %p)", filp, vma);
    
    vma->vm_ops = &ipacNopageVmOps;
    vma->vm_flags |= VM_RESERVED | VM_IO;
    vma->vm_private_data = node_p;
#ifndef ALLOW_CACHED_USER_READS
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
#endif
    ipacVmaOpen(vma);

    node_p->vm_start = (u8*)vma->vm_start;

    PRINTD(DBG_FUNC_CALLS, "ipacMmap returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacOpen
 * Inputs        : inode - inode associated with the device
 *                 filp  - user's file descriptor info
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : The open() call handler for the device.
 *
 *****************************************************************************/
/*
 * Open a keystream device node. This uses the minor number of the device node
 * to find the channel to associate with. The device is marked as unconfigured
 * until the client has set the direction using an ioctl call
 */
static int ipacOpen( struct inode  *inode,
                     struct file   *filp )
{
    int              minor      = iminor( inode );
    unsigned         minor_base = MINOR( ipacDevice.devno );
    unsigned         channel    = minor - minor_base;
    IpacDeviceNode  *node_p     = NULL;
    int              contextId  = 0;
    IpacRequest     *buffer     = NULL;
    int              ret        = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacOpen(%p, %p)", inode, filp);
    
    if ( channel >= IPAC_MAX_DRIVER_NODES )
    {
        PRINTD(DBG_NOTICE, "Invalid device number %d is > %d", channel, IPAC_MAX_DRIVER_NODES);
        ret = -ENODEV;
    }
    else
    {
        node_p = &ipacNodes[ channel ];
    
        spin_lock_bh( &node_p->lock );
    
        if ( node_p->useCount != 0)
        {
            spin_unlock_bh( &node_p->lock );
            PRINTD(DBG_NOTICE, "Device number %d is already in use", channel);
            ret = -EBUSY;
        }
        else
        {
            node_p->useCount = 1;
            
            for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
            {
                node_p->context[contextId].key_p    = NULL;
                node_p->context[contextId].oldKey_p = NULL;
            }
        
            node_p->firstRequest_p = node_p->startOfPool_p;
            node_p->lastRequest_p  = node_p->startOfPool_p;
        
            node_p->requestsPending = 0;

            buffer = (IpacRequest*)(node_p->firstRequest_p);
            buffer->state = IPAC_REQUEST_FINAL;
        
            /* Odd channels are for downlink, even for uplink */    
            if (channel & 1)
            {
                node_p->direction = IPAC_DOWNLINK;
            }
            else
            {
                node_p->direction = IPAC_UPLINK;
            }
            
            spin_unlock_bh( &node_p->lock );
            
            ipacFlushRequestInfo(buffer);
            
            filp->private_data = node_p;
        }
    }

    PRINTD(DBG_FUNC_CALLS, "ipacOpen returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacRelease
 * Inputs        : inode - inode associated with the device
 *                 filp  - user's file descriptor info
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : The close() call handler for the device. 
 *
 *****************************************************************************/
static int ipacRelease( struct inode  *inode,
                        struct file   *filp )
{
    /* Allow at least 200 ms to complete, we get 2ms for each 1ms wait
     * due to the timer granularity.
     */
    const int        RETRY_LIMIT = 100;

    IpacDeviceNode  *node_p    = filp->private_data;
    int              contextId = 0;
    int              ret       = 0;
    int              busy      = 0;
    int              retries   = RETRY_LIMIT;
    int              unchanged = 0;
    u32              pending   = 0;
    
    PRINTD(DBG_FUNC_CALLS, "ipacRelease(%p, %p)", inode, filp);
    
    spin_lock_bh( &node_p->lock );

    /* Wait till any last active IOCTL call has been completed and all the
     * active cipher requests for this node have completed.
     */
    node_p->releasingNode = 1;

    do
    {
        busy = 0;

        retries--;

        if (node_p->inIoctlCall)
        {
            pending = 0;
            unchanged = 0;
            busy = 1;
        }
        else
        {
            u32 requestCount = 0;

            for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
            {
                IpacContext   *context_p = &(node_p->context[contextId]);
                
                if (context_p->oldCipherActiveRequests || context_p->cipherActiveRequests)
                {
                    /* Have to wait for at least one request to complete */
                    busy = 1;

                    requestCount += context_p->oldCipherActiveRequests + context_p->cipherActiveRequests;
                }
            }

            if (pending == requestCount)
            {
                unchanged++;
            }
            else
            {
                unchanged = 0;
                pending = requestCount;
            }
        }

        if (busy)
        {
            /* Wait for activity to stop.  This shouldn't take long. */
            spin_unlock_bh( &node_p->lock );

            /* Keep polling, because interrupts alone will not handle
             * the last few results, unless there happen to be exactly 64.
             */
            ipacProcessCompletedPdus();
            ipacProcessQueuedPdus();

            msleep(1);

            spin_lock_bh( &node_p->lock );
        }
        
    } while (busy && (retries > 0));

    /* If the release had stalled then we should try to close the driver anyway. */
    if (busy && (unchanged > (RETRY_LIMIT/2)))
    {
        PRINTD(DBG_ERROR, "Forcing close with %d stalled requests.", pending);
        busy = 0;
    }

    if (busy)
    {
        if (pending == 0)
        {
            PRINTD(DBG_ERROR, "Unable to close due to IOCTL requests.");
        }
        else
        {
            PRINTD(DBG_ERROR, "Unable to close due to %d active requests.", pending);
        }
        ret = -EBUSY;
    }
    else
    {
        /* All requests are complete, and new IOCTL calls are being bounced,
         * so there can be no more requests and it's safe to release all the
         * cipher contexts.
         */
        for (contextId=0; contextId<IPAC_NUM_CONTEXTS; ++contextId)
        {
            IpacContext  *context_p = &(node_p->context[contextId]);
        
            context_p->cipherActiveRequests = 0;
            context_p->oldCipherActiveRequests = 0;

            if (context_p->key_p != NULL)
            {
                PRINTD(DBG_CIPHER, "Freeing current cipher %p in release", context_p->key_p);
                context_p->key_p    = NULL;
            }
        
            if (context_p->oldKey_p != NULL)
            {
                PRINTD(DBG_CIPHER, "Freeing old cipher %p in release", context_p->oldKey_p);
                context_p->oldKey_p = NULL;
            }
        }

        /* Allow this node to be reopened. */
        node_p->useCount = 0;
    }

    node_p->releasingNode = 0;
    spin_unlock_bh( &node_p->lock );

    PRINTD(DBG_FUNC_CALLS, "ipacRelease returned %d", ret);
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacRequestAndMap
 * Inputs        : resource - resource to map
 *                 name     - name of driver mapping resource
 * Outputs       : -
 * Returns       : pointer to memory mapped region or NULL
 *
 * Description   : Request and map a memory region for I/O transfers
 *
 *****************************************************************************/
static void __iomem *ipacRequestAndMap( struct resource  *resource,
                                        const char       *name )
{
    unsigned long     size = resource->end - resource->start + 1;
    void __iomem     *ret  = NULL;
    struct resource  *req  = request_mem_region( resource->start, size, name );

    PRINTD(DBG_FUNC_CALLS, "ipacRequestAndMap(%p,%s)", resource, name);
    
    if ( !req )
        goto req_fail;

    ret = ioremap( req->start, size );
    if ( !ret )
    {
        PRINTD(DBG_ERROR, "ioremap(%x, %ld) failed", req->start, size);
        goto remap_failed;
    }

    goto out;

remap_failed:
    release_resource( req );

req_fail:

out:
    PRINTD(DBG_FUNC_CALLS, "ipacRequestAndMap returned %p", ret);
    return ret;
}

/******************************************************************************
 * Function Name : ipacReleaseAndUnmap
 * Inputs        : resource - resource to unmap
 *                 iomem    - user's file descriptor info
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Unmap and release a memory region used for I/O transfers
 *
 *****************************************************************************/
static void ipacReleaseAndUnmap( struct resource  *resource,
                                 void __iomem     *iomem )
{
    unsigned long  size = resource->end - resource->start + 1;
    
    PRINTD(DBG_FUNC_CALLS, "ipacReleaseAndUnmap(%p, %p)", resource, iomem);
    
    iounmap( iomem );
    release_mem_region( resource->start, size );
    
    PRINTD(DBG_FUNC_CALLS, "ipacReleaseAndUnmap out");
}

/******************************************************************************
 * Function Name : ipacIrq
 * Inputs        : irq  - interrupt that has fired
 *                 dev  - device that registered the interrupt
 * Outputs       : -
 * Returns       : IRQ_HANDLED
 *
 * Description   : Interrupt handler for the cipher engine.  Process all
 *                 completed PDUs and see if any more can be submitted.
 *
 *****************************************************************************/
static irqreturn_t ipacIrq( int    irq,
                            void  *dev )
{
    u32  irq_stat            = ipacRegRead( SPA_IRQ_STAT_REG_OFFSET );
    u32  entryActiveRequests = ipacDevice.numActiveRequests;
    u32  entryQueuedRequests = ipacDevice.numQueuedRequests;

    PRINTD(DBG_FUNC_CALLS, "ipacIrq(%d, %p)", irq, dev);

    /* Process the completed packets. */
    ipacProcessCompletedPdus();

    /* Clear the interrupts. */
    ipacRegWrite( SPA_IRQ_STAT_REG_OFFSET, irq_stat );

    /* Take the chance to submit any PDUs that are already queued */
    ipacProcessQueuedPdus();

    PRINTD(DBG_IRQ, "ipacIrq: active %d to %d, queued %d to %d",
                     entryActiveRequests, ipacDevice.numActiveRequests,
                     entryQueuedRequests, ipacDevice.numQueuedRequests);
    
    
    PRINTD(DBG_FUNC_CALLS, "ipacIrq returned %d", IRQ_HANDLED);
    
    return IRQ_HANDLED;
}

/******************************************************************************
 * Function Name : ipacProbe
 * Inputs        : pdev - pointer to platform device to activate
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Called when the module is loaded.
 *
 *****************************************************************************/
static int ipacProbe( struct platform_device  *pdev )
{
    IpacDeviceNode   *node_p = NULL;
    IpacRequest      *buffer = NULL;
    int               ret     = -EINVAL;
    int               i;
    struct resource  *mem_resource;
    struct resource  *irq;
    const unsigned    dst_ddt_len = (IPAC_NUM_CIPHER_ENGINE_REQUESTS)*sizeof( IpacDdtContext );
    const unsigned    src_ddt_len = sizeof( IpacDdtContext );

    PRINTD(DBG_FUNC_CALLS, "ipacProbe(%p)", pdev);

    /* First of all set up the common device corresponding to the cipher HW. */
    
    ipacDevice.dev = &pdev->dev;

    mem_resource = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    if ( !mem_resource )
    {
        PRINTD( DBG_ERROR, "no memory resource for crypto HW" );
        goto out;
    }

    irq = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );
    if ( !irq )
    {
        PRINTD( DBG_ERROR, "no IRQ resource for crypto HW" );
        goto out;
    }

    /* Enable the clock to the cipher engine.  L2 engine is clock spacc3 */
    ipacDevice.clk = clk_get_sys( "picoxcell-l2", NULL );
    if ( !ipacDevice.clk || IS_ERR( ipacDevice.clk ) )
    {
        PRINTD( DBG_ERROR, "clk unavailable" );
        goto out;
    }

    if ( clk_enable( ipacDevice.clk ) )
    {
        PRINTD( DBG_ERROR, "unable to enable clk" );
        goto clk_en_fail;
    }

    /* Map the L2 cipher engine's registers */
    ipacDevice.crypto_regs = ipacRequestAndMap( mem_resource, pdev->name );
    if ( !ipacDevice.crypto_regs )
    {
        PRINTD( DBG_ERROR, "register memory map failed" );
        goto reg_map_fail;
    }
    ipacDevice.cipher_ctx_base = ipacDevice.crypto_regs + SPA_CIPH_KEY_BASE_REG_OFFSET;
    ipacDevice.hash_key_base   = ipacDevice.crypto_regs + SPA_HASH_KEY_BASE_REG_OFFSET;

    /* Setup and configure interrupts from the L2 engine */
    ret = request_irq( irq->start, ipacIrq, 0, pdev->name, &ipacDevice );
    if ( ret )
    {
        PRINTD( DBG_ERROR, "failed to request IRQ\n" );
        goto irq_fail;
    }

    /* Configure the interrupts. We only use the STAT_CNT interrupt as we only
     * submit a new packet for processing when we complete another in the
     * queue. This minimizes time spent in the interrupt handler. */
    ipacRegWrite( SPA_IRQ_CTRL_REG_OFFSET,  (IPAC_NUM_CIPHER_ENGINE_REQUESTS/2) << SPA_IRQ_CTRL_STAT_CNT_OFFSET );
    ipacRegWrite( SPA_IRQ_EN_REG_OFFSET, SPA_IRQ_EN_STAT_EN | SPA_IRQ_EN_GLBL_EN );

    /* Map DMA memory to hold the data descriptor tables defining the destinations.
     * We have one possible destination for each possible request that can be active.
     */
    ipacDevice.dst_ddt_buf = dma_alloc_coherent(ipacDevice.dev, dst_ddt_len, &ipacDevice.dst_ddt_buf_phys, GFP_KERNEL);
    if ( !ipacDevice.dst_ddt_buf )
    {
        ret = -ENOMEM;
        goto dst_ddt_failed;
    }
    for (i=0; i<IPAC_NUM_CIPHER_ENGINE_REQUESTS; ++i)
    {
        ipacDevice.dst_ddt_buf[i].ddt.p1   = 0;
        ipacDevice.dst_ddt_buf[i].ddt.len1 = 0;
    }

    spin_lock_init( &ipacDevice.hw_lock );
    
    /* Map DMA memory to hold a data descriptor table defining the source.
     * We only need a single source, since we always pass in zeros to get a
     * keystream out that we XOR with the data, so get a zeroed page and
     * map that directly.
     */
    ipacDevice.zeroed_page = ( void * )get_zeroed_page( GFP_KERNEL );
    if ( !ipacDevice.zeroed_page )
    {
        PRINTD( DBG_ERROR, "failed to allocate a zeroed page (%s)\n", pdev->name );
        goto no_zeroed_mem;
    }
    
    sg_init_table( &ipacDevice.zeroed_list, 1 );
    sg_set_buf( &ipacDevice.zeroed_list, ipacDevice.zeroed_page, 4096 );
    dma_map_sg( ipacDevice.dev, &ipacDevice.zeroed_list, 1, DMA_TO_DEVICE );

    ipacDevice.src_ddt_buf = dma_alloc_coherent(ipacDevice.dev, src_ddt_len, &ipacDevice.src_ddt_buf_phys, GFP_KERNEL);
    if ( !ipacDevice.src_ddt_buf )
    {
        ret = -ENOMEM;
        goto src_ddt_failed;
    }

    ipacDevice.src_ddt_buf->ddt.p0   = sg_dma_address( &ipacDevice.zeroed_list );
    ipacDevice.src_ddt_buf->ddt.len0 = sg_dma_len( &ipacDevice.zeroed_list );
    ipacDevice.src_ddt_buf->ddt.p1   = 0;
    ipacDevice.src_ddt_buf->ddt.len1 = 0;
    
    platform_set_drvdata( pdev, &ipacDevice );

    /* Now we set up a couple of nodes.  One for downlink and
     * another for uplink.
     */
    for (i=0; i<IPAC_MAX_DRIVER_NODES; ++i)
    {
        memset(&ipacNodes[i], 0, sizeof(IpacDeviceNode));

        spin_lock_init( &ipacNodes[i].lock );
        strcpy(ipacNodes[i].name, "reqx");
        ipacNodes[i].name[3] = '0' + i;
    }
    
    cdev_init( &ipacDevice.cdev, &ipacFileOps );
    ipacDevice.cdev.owner = THIS_MODULE;
    ipacDevice.cdev.ops   = &ipacFileOps;

    ipacDevice.devno = MKDEV(IPAC_DEVICE_MAJOR, 0);

    ret = register_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES, "ipa_crypto" );
    if ( ret )
    {
        PRINTD(DBG_ERROR, "failed to allocate dev node range" );
        goto cdev_register_failed;
    }

    ipacDevice.sysfs_class = class_create( THIS_MODULE, "ipa_crypto" );
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        ipacNodes[i].dev_p =  device_create( ipacDevice.sysfs_class, NULL,
                                             MKDEV( MAJOR( ipacDevice.devno ),
                                             MINOR( ipacDevice.devno ) + i ), NULL,
                                             "ipa_crypto%u", i );
        if (ipacNodes[i].dev_p == NULL)
        {
            PRINTD(DBG_ERROR, "failed to create device %d", ret );
            ret = -ENOMEM;
            goto device_creation_failed;
        }

        ipacNodes[i].dev_p->coherent_dma_mask = 0xFFFFFFFF;
        ipacNodes[i].dev_p->dma_mask = &(ipacNodes[i].dev_p->coherent_dma_mask);
    }


    ret = cdev_add( &ipacDevice.cdev, ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
    if ( ret )
    {
        PRINTD(DBG_ERROR, "failed to add cdev" );
        goto cdev_add_failed;
    }
    
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        node_p->startOfPool_p = kmalloc(IPAC_KEYSTREAM_BUFFER_SIZE, GFP_KERNEL | __GFP_DMA);
        if ( node_p->startOfPool_p == NULL )
        {
            ret = -ENOMEM;
            PRINTD(DBG_ERROR, "failed to allocate memory for pool" );
            goto dma_alloc_failed;
        }

        node_p->startOfDma_p = dma_map_single(node_p->dev_p, node_p->startOfPool_p,
                                              IPAC_KEYSTREAM_BUFFER_SIZE, DMA_FROM_DEVICE);
        if ( node_p->startOfDma_p == 0 )
        {
            ret = -ENOMEM;
            PRINTD(DBG_ERROR, "failed to map DMA memory for pool" );
            goto dma_alloc_failed;
        }
        node_p->dmaOffset = (u32)node_p->startOfPool_p - node_p->startOfDma_p;

        node_p->endOfPool_p    = node_p->startOfPool_p + IPAC_KEYSTREAM_BUFFER_SIZE;
        node_p->firstRequest_p = node_p->startOfPool_p;
        node_p->lastRequest_p  = node_p->startOfPool_p;
        
        node_p->requestsPending = 0;

        buffer = (IpacRequest*)(node_p->firstRequest_p);
        buffer->state = IPAC_REQUEST_FINAL;
        ipacFlushRequestInfo(buffer);
    }

    /* Make sure there are no stale PDUs waiting in the cipher engine */
    ipacCleanupCompletedPdus();

    /* Set up various registers whose value never changes for F8. */
    ipacRegWrite( SPA_SRC_PTR_REG_OFFSET, ( u32 )ipacDevice.src_ddt_buf_phys );
    ipacRegWrite( SPA_OFFSET_REG_OFFSET, 0 );
    ipacRegWrite( SPA_ICV_OFFSET_REG_OFFSET(IPAC_CIPHER_KEY_VERSION), 0 );
    ipacRegWrite( SPA_ICV_LEN_REG_OFFSET(IPAC_CIPHER_KEY_VERSION), 0 );
    ipacRegWrite( SPA_AUX_INFO_REG_OFFSET(IPAC_CIPHER_KEY_VERSION), 0 );
    ipacRegWrite( SPA_AAD_LEN_REG_OFFSET, 0 );

    ret = 0;
    goto out;



dma_alloc_failed:
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        if (node_p->startOfDma_p)
        {
            dma_unmap_single( node_p->dev_p, node_p->startOfDma_p, IPAC_KEYSTREAM_BUFFER_SIZE, DMA_FROM_DEVICE );
            node_p->startOfDma_p = 0;
        }

        if (node_p->startOfPool_p)
        {
            kfree(node_p->startOfPool_p);
            node_p->startOfPool_p = NULL;
        }
    }
    
    cdev_del( &ipacDevice.cdev );

cdev_add_failed:

device_creation_failed:
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        if (ipacNodes[i].dev_p)
        {
            device_destroy( ipacDevice.sysfs_class,
                            MKDEV( MAJOR( ipacDevice.devno ),
                            MINOR( ipacDevice.devno ) + i ) );
        }
    }
    class_destroy( ipacDevice.sysfs_class );
    
    unregister_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
    
cdev_register_failed:
    dma_free_coherent( ipacDevice.dev, src_ddt_len, ipacDevice.src_ddt_buf, ipacDevice.src_ddt_buf_phys );

src_ddt_failed:
    dma_unmap_sg(ipacDevice.dev, &ipacDevice.zeroed_list, 1, DMA_TO_DEVICE );
    free_page( ( unsigned long )ipacDevice.zeroed_page );
    
no_zeroed_mem:
    dma_free_coherent( ipacDevice.dev, dst_ddt_len, ipacDevice.dst_ddt_buf, ipacDevice.dst_ddt_buf_phys );

dst_ddt_failed:
    free_irq( irq->start, &ipacDevice );
    
irq_fail:
    ipacReleaseAndUnmap( mem_resource, ipacDevice.crypto_regs );
    
reg_map_fail:
    clk_disable( ipacDevice.clk );

clk_en_fail:
    clk_put( ipacDevice.clk );
    
out:
    PRINTD(DBG_FUNC_CALLS, "ipacProbe returned %d", ret);

    return ret;
}

/******************************************************************************
 * Function Name : ipacExit
 * Inputs        : pdev - pointer to platform device to remove
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Cleanup when the module is unloaded
 *
 *****************************************************************************/
static int ipacRemove( struct platform_device  *pdev )
{
    IpacDeviceNode   *node_p       = NULL;
    struct resource  *mem_resource = platform_get_resource( pdev, IORESOURCE_MEM, 0 );
    struct resource  *irq          = platform_get_resource( pdev, IORESOURCE_IRQ, 0 );
    const unsigned    dst_ddt_len  = (IPAC_NUM_CIPHER_ENGINE_REQUESTS)*sizeof( IpacDdtContext );
    const unsigned    src_ddt_len  = sizeof( IpacDdtContext );
    int               i;
    int               ret = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacRemove(%p)", pdev);

    /* TODO Will have to clean up all allocated memory before unloading the driver */

    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        node_p = &ipacNodes[i];
        
        if (node_p->startOfDma_p)
        {
            dma_unmap_single( node_p->dev_p, node_p->startOfDma_p, IPAC_KEYSTREAM_BUFFER_SIZE, DMA_FROM_DEVICE );
            node_p->startOfDma_p = 0;
        }

        if (node_p->startOfPool_p)
        {
            kfree(node_p->startOfPool_p);
            node_p->startOfPool_p = NULL;
        }
    }
    
    cdev_del( &ipacDevice.cdev );
    
    for ( i = 0; i < IPAC_MAX_DRIVER_NODES; ++i )
    {
        device_destroy( ipacDevice.sysfs_class,
                        MKDEV( MAJOR( ipacDevice.devno ),
                               MINOR( ipacDevice.devno ) + i ) );
    }
    class_destroy( ipacDevice.sysfs_class );

    unregister_chrdev_region( ipacDevice.devno, IPAC_MAX_DRIVER_NODES );
    
    dma_free_coherent( ipacDevice.dev, src_ddt_len, ipacDevice.src_ddt_buf, ipacDevice.src_ddt_buf_phys );
    
    dma_unmap_sg(ipacDevice.dev, &ipacDevice.zeroed_list, 1, DMA_TO_DEVICE );
    free_page( ( unsigned long )ipacDevice.zeroed_page );
    
    dma_free_coherent( ipacDevice.dev, dst_ddt_len, ipacDevice.dst_ddt_buf, ipacDevice.dst_ddt_buf_phys );
    
    free_irq( irq->start, &ipacDevice );
    
    ipacReleaseAndUnmap( mem_resource, ipacDevice.crypto_regs );
    
    clk_disable( ipacDevice.clk );
    clk_put( ipacDevice.clk );
    
    PRINTD(DBG_FUNC_CALLS, "ipacRemove out");

    return ret;
}

#ifdef CONFIG_PM
/******************************************************************************
 * Function Name : ipacSuspend
 * Inputs        : pdev  - pointer to device to suspend
 *                 state - suspend state to enter
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Called when the module is suspended.
 *
 *****************************************************************************/
static int ipacSuspend( struct platform_device  *pdev,
                        pm_message_t             state )
{
    IpacDevice  *ipacDevice_p = platform_get_drvdata( pdev );
    int          ret          = 0;

    PRINTD(DBG_FUNC_CALLS, "ipacSuspend(%p, %d)", pdev, state.event);
    
    /*
     * We only support standby mode. All we have to do is gate the clock to
     * the spacc. The hardware will preserve state until we turn it back
     * on again.
     */
    clk_disable( ipacDevice_p->clk );

    PRINTD(DBG_FUNC_CALLS, "ipacSuspend returned 0");
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacResume
 * Inputs        : pdev - pointer to device to resume
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Called when the module is resumed after suspension.
 *
 *****************************************************************************/
static int ipacResume( struct platform_device *pdev )
{
    IpacDevice  *ipacDevice_p = platform_get_drvdata( pdev );
    int          ret;
    
    PRINTD(DBG_FUNC_CALLS, "ipacResume(%p)", pdev);

    ret = clk_enable( ipacDevice_p->clk );
    
    PRINTD(DBG_FUNC_CALLS, "ipacResume returned %d", ret);
    
    return ret;
}
#else /* CONFIG_PM */
#define ipacSuspend   NULL
#define ipacResume    NULL
#endif /* CONFIG_PM */

static struct platform_driver ipacL2Driver = {
    .probe      = ipacProbe,
    .remove     = ipacRemove,
    .suspend    = ipacSuspend,
    .resume     = ipacResume,
    .driver = {
        .name = "picoxcell-l2-v2",
    },
};

/******************************************************************************
 * Function Name : ipacInit
 * Inputs        : -
 * Outputs       : -
 * Returns       : status 0 => OK
 *
 * Description   : Called when the module is loaded.
 *
 *****************************************************************************/
static int ipacInit( void )
{
    int  ret;
    
    PRINTD(DBG_FUNC_CALLS, "ipacInit()");
    
    ret = platform_driver_register( &ipacL2Driver );
    if ( ret )
    {
        PRINTD( DBG_ERROR, "failed to register platform driver" );
    }

    PRINTD(DBG_FUNC_CALLS, "ipacInit out");
    
    return ret;
}

/******************************************************************************
 * Function Name : ipacExit
 * Inputs        : -
 * Outputs       : -
 * Returns       : -
 *
 * Description   : Cleanup when the module is unloaded
 *
 *****************************************************************************/
static void ipacExit( void )
{
    PRINTD(DBG_FUNC_CALLS, "ipacExit()");
    
    platform_driver_unregister( &ipacL2Driver );
    
    PRINTD(DBG_FUNC_CALLS, "ipacExit out");
}

module_init( ipacInit );
module_exit( ipacExit );

MODULE_AUTHOR("ip.access");
MODULE_DESCRIPTION("ipacrypto user interface driver for f8(kasumi)");
MODULE_LICENSE( "GPL" );
