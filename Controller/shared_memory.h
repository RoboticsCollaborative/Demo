#ifndef SHARED_MEMORY_H
#define SHARED_MEMORY_H

#include "shared_data.h"

#define SHARED_IN "/shared_input"
#define SHARED_OUT "/shared_output"


int openSharedMemory(char *shm_name, void **p);
int ticket_init(ticket_lock_t *ticket);
int ticket_lock(ticket_lock_t *ticket); 
int ticket_unlock(ticket_lock_t *ticket); 

#endif /* SHARED_MEMORY_H */
