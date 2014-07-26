#ifndef URG_RING_BUFFER_H
#define URG_RING_BUFFER_H

typedef struct
{
	char *buffer;
	int buffer_size;
	int first;
	int last;
} ring_buffer_t;

/**
 * @brief Initialize the ring buffer
 * 
 * @param ring pointer to a ring buffer
 * @param buffer a buffer
 * @param shift_length 
 */
extern void ring_initialize(ring_buffer_t *ring, char *buffer, const int shift_length);

extern void ring_clear(ring_buffer_t *ring);

extern int ring_size(const ring_buffer_t *ring);

extern int ring_capacity(const ring_buffer_t *ring);

extern int ring_write(ring_buffer_t *ring, const char *data, int size);

extern int ring_read(ring_buffer_t *ring, char *buffer, int size);


#endif /* ! RING_BUFFER_H */
