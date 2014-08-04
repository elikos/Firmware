#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

#ifdef __cplusplus
#define EXTERN_C       extern "C"
#define EXTERN_C_BEGIN extern "C" {
#define EXTERN_C_END   }
#else
#define EXTERN_C       /* Nothing */
#define EXTERN_C_BEGIN /* Npthing */
#define EXTERN_C_END   /* Nothing */
#endif

typedef struct
{
	char *buffer;
	int buffer_size;
	int first;
	int last;
} ring_buffer_t;

EXTERN_C_BEGIN
/**
 * @brief Initialize the ring buffer
 * 
 * @param ring 			Pointer to a ring buffer
 * @param buffer 	A buffer
 * @param shift_length 	Will become size of buffer, 
 */
void ring_initialize(ring_buffer_t *ring, char *buffer, const int shift_length);

void ring_clear(ring_buffer_t *ring);

int ring_size(const ring_buffer_t *ring);

int ring_capacity(const ring_buffer_t *ring);

int ring_write(ring_buffer_t *ring, const char *data, int size);

int ring_read(ring_buffer_t *ring, char *buffer, int size);

EXTERN_C_END

#endif /* ! RING_BUFFER_H */
