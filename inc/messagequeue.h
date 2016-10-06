#ifndef MESSAGEQUEUE_H
#define MESSAGEQUEUE_H

#define MSG_POTITURN 1
#define MSG_BUTTONPRESS 2
#define MSG_LEDRING 3
#define MSG_CHANNELLED 4
#define MSG_SEGMENT 5
#define MSG_LEDVALUES 6

#define MSG_POTILEFT 16
#define MSG_POTIRIGHT 32


uint8_t message_create(uint8_t, uint8_t);
void messagequeue_init(void);
void messagequeue_update_i2c_buffers(void);
void messagequeue_i2crx_handler(void);

extern volatile uint8_t GL_messagequeue[32][2];
extern volatile uint8_t GL_currentmessage;
extern volatile uint8_t GL_firstmessage;
extern volatile uint8_t GL_messagequeue_elements;
extern volatile uint8_t GL_i2c_messagequeue_busy;

#endif // MESSAGEQUEUE_H
