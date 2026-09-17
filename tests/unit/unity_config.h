#ifndef UNITY_CONFIG_H
#define UNITY_CONFIG_H

/* Unity framework configuration */

#define UNITY_INCLUDE_PRINT_FORMATTED 1
#define UNITY_SUPPORT_VARIADIC_MACROS 1

/* Output */
#define UNITY_OUTPUT_CHAR(c) putchar(c)
#define UNITY_OUTPUT_START() do {} while(0)
#define UNITY_OUTPUT_FINISH() do {} while(0)

#endif /* UNITY_CONFIG_H */
