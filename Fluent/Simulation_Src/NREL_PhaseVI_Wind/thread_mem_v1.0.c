/* Purpose: A general purpose thread memory allocator for FLUENT.  
   Author : H. Krishnan

   Description:
   This file provides functionality for allocating user-defined memory at
   threads. Functionalities provided are allocatng a memory (identified by a
   string) to a thread, getting the memory for use, and freeing the memory.

   void Alloc_Thread_Memory(Thread *t, char *mem_name, size_t size)
      - Will allocate memory of length size*SV_LENGTH(t)

   void Get_Thread_Memory(Thread *t, char *mem_name)
      - Return allocated memory in thread t of name mem_name

   void Free_Thread_Memory(Thread *t, char *mem_name)
      - Free memory named mem_name in thread t

 */

/*---------------------------------------------------------------------------*/
/* v1.0 replaces CX_Malloc and CX_Free with malloc and free since the previous 
        have trouble on XP in parallel. Also fixed the issue of Krishnan's 
        original UDF of allocating too much memory                           */
/*---------------------------------------------------------------------------*/

#include "global.h"
#include "mem.h"

typedef struct thread_mem_struct {
    int thread_id;
    char mem_name[64];
    void *mem;
    struct thread_mem_struct *next;
} Thread_Mem_Struct;

Thread_Mem_Struct *my_mem = NULL;

#define my_mem_loop(m)for(m = my_mem; m != NULL; m = m->next)

void *
Get_Thread_Memory(Thread *t, char *mem_name)
{
    Thread_Mem_Struct *m;
    int id = THREAD_ID(t);
    my_mem_loop(m)
      {
        if ((id == m->thread_id) &&
            STREQ(mem_name, m->mem_name))
          return m->mem;
      }
    return NULL;
}

static void
add_thread_memory(Thread *t, char *mem_name, void *mem)
{
    Thread_Mem_Struct *new_mem = malloc(sizeof(Thread_Mem_Struct));
    if (NULL == new_mem)
      Error("Unable to add new thread memory");

    strncpy(new_mem->mem_name,mem_name,64);
    new_mem->mem = mem;
    new_mem->thread_id = THREAD_ID(t);
    new_mem->next = my_mem;
    my_mem = new_mem;

}

void
Alloc_Thread_Memory(Thread *t, char *mem_name, size_t size)
{

    void *new_mem;

    if (NNULLP(new_mem = Get_Thread_Memory(t,mem_name)))
      {
        Message("Warning: Memory Already Allocated");
        return;
      }
    new_mem = malloc(size);
    if (NULL == new_mem)
      Error("Unable to add new thread memory");
    add_thread_memory(t, mem_name, new_mem);
    return;
}

void
Free_Thread_Memory(Thread *t, char *mem_name)
{
    Thread_Mem_Struct *m, *pre;
    pre = NULL;
    my_mem_loop(m)
      {
        if ((THREAD_ID(t) == m->thread_id) &&
            STREQ(mem_name, m->mem_name))
          {
            if (NNULLP(pre))
              pre->next = m->next;

            if (m == my_mem)
              my_mem = m->next;

            free(m->mem);
            free(m);
            break;
          }
        pre = m;
      }
}
