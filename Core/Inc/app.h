#ifndef APP_H
#define APP_H

/* Call once from main() after all MX_..._Init() functions. */
void App_Init(void);

/* Call repeatedly from the while(1) loop in main(). */
void App_Loop(void);

#endif /* APP_H */
