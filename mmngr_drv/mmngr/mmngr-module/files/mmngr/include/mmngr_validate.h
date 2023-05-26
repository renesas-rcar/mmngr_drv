#ifndef __MMNGR_VALIDATE_H__
#define __MMNGR_VALIDATE_H__

#if defined(MMNGR_VALIDATE)
#define MMNGR_ADDRESS_VALIDATION
#endif

bool mmngr_validate_phys_addr(phys_addr_t phys_addr, u64 size);

#endif /* __MMNGR_VALIDATE_H__ */
