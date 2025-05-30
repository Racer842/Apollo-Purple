/* auto-generated by gen_syscalls.py, don't edit */

#ifndef ZEPHYR_SYSCALL_LIST_H
#define ZEPHYR_SYSCALL_LIST_H

#define K_SYSCALL_DEVICE_GET_BINDING 0
#define K_SYSCALL_DEVICE_GET_BY_DT_NODELABEL 1
#define K_SYSCALL_DEVICE_INIT 2
#define K_SYSCALL_DEVICE_IS_READY 3
#define K_SYSCALL_ENTROPY_GET_ENTROPY 4
#define K_SYSCALL_GPIO_GET_PENDING_INT 5
#define K_SYSCALL_GPIO_PIN_CONFIGURE 6
#define K_SYSCALL_GPIO_PIN_GET_CONFIG 7
#define K_SYSCALL_GPIO_PIN_INTERRUPT_CONFIGURE 8
#define K_SYSCALL_GPIO_PORT_CLEAR_BITS_RAW 9
#define K_SYSCALL_GPIO_PORT_GET_DIRECTION 10
#define K_SYSCALL_GPIO_PORT_GET_RAW 11
#define K_SYSCALL_GPIO_PORT_SET_BITS_RAW 12
#define K_SYSCALL_GPIO_PORT_SET_MASKED_RAW 13
#define K_SYSCALL_GPIO_PORT_TOGGLE_BITS 14
#define K_SYSCALL_I2C_CONFIGURE 15
#define K_SYSCALL_I2C_GET_CONFIG 16
#define K_SYSCALL_I2C_RECOVER_BUS 17
#define K_SYSCALL_I2C_TARGET_DRIVER_REGISTER 18
#define K_SYSCALL_I2C_TARGET_DRIVER_UNREGISTER 19
#define K_SYSCALL_I2C_TRANSFER 20
#define K_SYSCALL_K_BUSY_WAIT 21
#define K_SYSCALL_K_CONDVAR_BROADCAST 22
#define K_SYSCALL_K_CONDVAR_INIT 23
#define K_SYSCALL_K_CONDVAR_SIGNAL 24
#define K_SYSCALL_K_CONDVAR_WAIT 25
#define K_SYSCALL_K_EVENT_CLEAR 26
#define K_SYSCALL_K_EVENT_INIT 27
#define K_SYSCALL_K_EVENT_POST 28
#define K_SYSCALL_K_EVENT_SET 29
#define K_SYSCALL_K_EVENT_SET_MASKED 30
#define K_SYSCALL_K_EVENT_WAIT 31
#define K_SYSCALL_K_EVENT_WAIT_ALL 32
#define K_SYSCALL_K_FLOAT_DISABLE 33
#define K_SYSCALL_K_FLOAT_ENABLE 34
#define K_SYSCALL_K_FUTEX_WAIT 35
#define K_SYSCALL_K_FUTEX_WAKE 36
#define K_SYSCALL_K_IS_PREEMPT_THREAD 37
#define K_SYSCALL_K_MSGQ_ALLOC_INIT 38
#define K_SYSCALL_K_MSGQ_GET 39
#define K_SYSCALL_K_MSGQ_GET_ATTRS 40
#define K_SYSCALL_K_MSGQ_NUM_FREE_GET 41
#define K_SYSCALL_K_MSGQ_NUM_USED_GET 42
#define K_SYSCALL_K_MSGQ_PEEK 43
#define K_SYSCALL_K_MSGQ_PEEK_AT 44
#define K_SYSCALL_K_MSGQ_PURGE 45
#define K_SYSCALL_K_MSGQ_PUT 46
#define K_SYSCALL_K_MUTEX_INIT 47
#define K_SYSCALL_K_MUTEX_LOCK 48
#define K_SYSCALL_K_MUTEX_UNLOCK 49
#define K_SYSCALL_K_OBJECT_ACCESS_GRANT 50
#define K_SYSCALL_K_OBJECT_ALLOC 51
#define K_SYSCALL_K_OBJECT_ALLOC_SIZE 52
#define K_SYSCALL_K_OBJECT_RELEASE 53
#define K_SYSCALL_K_PIPE_ALLOC_INIT 54
#define K_SYSCALL_K_PIPE_BUFFER_FLUSH 55
#define K_SYSCALL_K_PIPE_CLOSE 56
#define K_SYSCALL_K_PIPE_FLUSH 57
#define K_SYSCALL_K_PIPE_GET 58
#define K_SYSCALL_K_PIPE_INIT 59
#define K_SYSCALL_K_PIPE_PUT 60
#define K_SYSCALL_K_PIPE_READ 61
#define K_SYSCALL_K_PIPE_READ_AVAIL 62
#define K_SYSCALL_K_PIPE_RESET 63
#define K_SYSCALL_K_PIPE_WRITE 64
#define K_SYSCALL_K_PIPE_WRITE_AVAIL 65
#define K_SYSCALL_K_POLL 66
#define K_SYSCALL_K_POLL_SIGNAL_CHECK 67
#define K_SYSCALL_K_POLL_SIGNAL_INIT 68
#define K_SYSCALL_K_POLL_SIGNAL_RAISE 69
#define K_SYSCALL_K_POLL_SIGNAL_RESET 70
#define K_SYSCALL_K_QUEUE_ALLOC_APPEND 71
#define K_SYSCALL_K_QUEUE_ALLOC_PREPEND 72
#define K_SYSCALL_K_QUEUE_CANCEL_WAIT 73
#define K_SYSCALL_K_QUEUE_GET 74
#define K_SYSCALL_K_QUEUE_INIT 75
#define K_SYSCALL_K_QUEUE_IS_EMPTY 76
#define K_SYSCALL_K_QUEUE_PEEK_HEAD 77
#define K_SYSCALL_K_QUEUE_PEEK_TAIL 78
#define K_SYSCALL_K_RESCHEDULE 79
#define K_SYSCALL_K_SCHED_CURRENT_THREAD_QUERY 80
#define K_SYSCALL_K_SEM_COUNT_GET 81
#define K_SYSCALL_K_SEM_GIVE 82
#define K_SYSCALL_K_SEM_INIT 83
#define K_SYSCALL_K_SEM_RESET 84
#define K_SYSCALL_K_SEM_TAKE 85
#define K_SYSCALL_K_SLEEP 86
#define K_SYSCALL_K_STACK_ALLOC_INIT 87
#define K_SYSCALL_K_STACK_POP 88
#define K_SYSCALL_K_STACK_PUSH 89
#define K_SYSCALL_K_STR_OUT 90
#define K_SYSCALL_K_THREAD_ABORT 91
#define K_SYSCALL_K_THREAD_CREATE 92
#define K_SYSCALL_K_THREAD_CUSTOM_DATA_GET 93
#define K_SYSCALL_K_THREAD_CUSTOM_DATA_SET 94
#define K_SYSCALL_K_THREAD_DEADLINE_SET 95
#define K_SYSCALL_K_THREAD_JOIN 96
#define K_SYSCALL_K_THREAD_NAME_COPY 97
#define K_SYSCALL_K_THREAD_NAME_SET 98
#define K_SYSCALL_K_THREAD_PRIORITY_GET 99
#define K_SYSCALL_K_THREAD_PRIORITY_SET 100
#define K_SYSCALL_K_THREAD_RESUME 101
#define K_SYSCALL_K_THREAD_STACK_ALLOC 102
#define K_SYSCALL_K_THREAD_STACK_FREE 103
#define K_SYSCALL_K_THREAD_STACK_SPACE_GET 104
#define K_SYSCALL_K_THREAD_SUSPEND 105
#define K_SYSCALL_K_THREAD_TIMEOUT_EXPIRES_TICKS 106
#define K_SYSCALL_K_THREAD_TIMEOUT_REMAINING_TICKS 107
#define K_SYSCALL_K_TIMER_EXPIRES_TICKS 108
#define K_SYSCALL_K_TIMER_REMAINING_TICKS 109
#define K_SYSCALL_K_TIMER_START 110
#define K_SYSCALL_K_TIMER_STATUS_GET 111
#define K_SYSCALL_K_TIMER_STATUS_SYNC 112
#define K_SYSCALL_K_TIMER_STOP 113
#define K_SYSCALL_K_TIMER_USER_DATA_GET 114
#define K_SYSCALL_K_TIMER_USER_DATA_SET 115
#define K_SYSCALL_K_UPTIME_TICKS 116
#define K_SYSCALL_K_USLEEP 117
#define K_SYSCALL_K_WAKEUP 118
#define K_SYSCALL_K_YIELD 119
#define K_SYSCALL_LOG_BUFFERED_CNT 120
#define K_SYSCALL_LOG_FILTER_SET 121
#define K_SYSCALL_LOG_FRONTEND_FILTER_SET 122
#define K_SYSCALL_LOG_PANIC 123
#define K_SYSCALL_LOG_PROCESS 124
#define K_SYSCALL_NET_ADDR_NTOP 125
#define K_SYSCALL_NET_ADDR_PTON 126
#define K_SYSCALL_NET_ETH_GET_PTP_CLOCK_BY_INDEX 127
#define K_SYSCALL_NET_IF_GET_BY_INDEX 128
#define K_SYSCALL_NET_IF_IPV4_ADDR_ADD_BY_INDEX 129
#define K_SYSCALL_NET_IF_IPV4_ADDR_LOOKUP_BY_INDEX 130
#define K_SYSCALL_NET_IF_IPV4_ADDR_RM_BY_INDEX 131
#define K_SYSCALL_NET_IF_IPV4_SET_GW_BY_INDEX 132
#define K_SYSCALL_NET_IF_IPV4_SET_NETMASK_BY_ADDR_BY_INDEX 133
#define K_SYSCALL_NET_IF_IPV4_SET_NETMASK_BY_INDEX 134
#define K_SYSCALL_NET_IF_IPV6_ADDR_ADD_BY_INDEX 135
#define K_SYSCALL_NET_IF_IPV6_ADDR_LOOKUP_BY_INDEX 136
#define K_SYSCALL_NET_IF_IPV6_ADDR_RM_BY_INDEX 137
#define K_SYSCALL_SPI_RELEASE 138
#define K_SYSCALL_SPI_TRANSCEIVE 139
#define K_SYSCALL_SYS_CLOCK_HW_CYCLES_PER_SEC_RUNTIME_GET 140
#define K_SYSCALL_SYS_CSRAND_GET 141
#define K_SYSCALL_SYS_RAND_GET 142
#define K_SYSCALL_UART_CONFIGURE 143
#define K_SYSCALL_UART_CONFIG_GET 144
#define K_SYSCALL_UART_DRV_CMD 145
#define K_SYSCALL_UART_ERR_CHECK 146
#define K_SYSCALL_UART_IRQ_ERR_DISABLE 147
#define K_SYSCALL_UART_IRQ_ERR_ENABLE 148
#define K_SYSCALL_UART_IRQ_IS_PENDING 149
#define K_SYSCALL_UART_IRQ_RX_DISABLE 150
#define K_SYSCALL_UART_IRQ_RX_ENABLE 151
#define K_SYSCALL_UART_IRQ_TX_DISABLE 152
#define K_SYSCALL_UART_IRQ_TX_ENABLE 153
#define K_SYSCALL_UART_IRQ_UPDATE 154
#define K_SYSCALL_UART_LINE_CTRL_GET 155
#define K_SYSCALL_UART_LINE_CTRL_SET 156
#define K_SYSCALL_UART_POLL_IN 157
#define K_SYSCALL_UART_POLL_IN_U16 158
#define K_SYSCALL_UART_POLL_OUT 159
#define K_SYSCALL_UART_POLL_OUT_U16 160
#define K_SYSCALL_UART_RX_DISABLE 161
#define K_SYSCALL_UART_RX_ENABLE 162
#define K_SYSCALL_UART_RX_ENABLE_U16 163
#define K_SYSCALL_UART_TX 164
#define K_SYSCALL_UART_TX_ABORT 165
#define K_SYSCALL_UART_TX_U16 166
#define K_SYSCALL_XTENSA_USER_FAULT 167
#define K_SYSCALL_ZEPHYR_FPUTC 168
#define K_SYSCALL_ZEPHYR_FWRITE 169
#define K_SYSCALL_ZEPHYR_READ_STDIN 170
#define K_SYSCALL_ZEPHYR_WRITE_STDOUT 171
#define K_SYSCALL_ZSOCK_ACCEPT 172
#define K_SYSCALL_ZSOCK_BIND 173
#define K_SYSCALL_ZSOCK_CLOSE 174
#define K_SYSCALL_ZSOCK_CONNECT 175
#define K_SYSCALL_ZSOCK_FCNTL_IMPL 176
#define K_SYSCALL_ZSOCK_GETHOSTNAME 177
#define K_SYSCALL_ZSOCK_GETPEERNAME 178
#define K_SYSCALL_ZSOCK_GETSOCKNAME 179
#define K_SYSCALL_ZSOCK_GETSOCKOPT 180
#define K_SYSCALL_ZSOCK_GET_CONTEXT_OBJECT 181
#define K_SYSCALL_ZSOCK_INET_PTON 182
#define K_SYSCALL_ZSOCK_IOCTL_IMPL 183
#define K_SYSCALL_ZSOCK_LISTEN 184
#define K_SYSCALL_ZSOCK_RECVFROM 185
#define K_SYSCALL_ZSOCK_RECVMSG 186
#define K_SYSCALL_ZSOCK_SENDMSG 187
#define K_SYSCALL_ZSOCK_SENDTO 188
#define K_SYSCALL_ZSOCK_SETSOCKOPT 189
#define K_SYSCALL_ZSOCK_SHUTDOWN 190
#define K_SYSCALL_ZSOCK_SOCKET 191
#define K_SYSCALL_ZSOCK_SOCKETPAIR 192
#define K_SYSCALL_ZVFS_POLL 193
#define K_SYSCALL_ZVFS_SELECT 194
#define K_SYSCALL_Z_ERRNO 195
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_0 196
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_1 197
#define K_SYSCALL_Z_LOG_MSG_SIMPLE_CREATE_2 198
#define K_SYSCALL_Z_LOG_MSG_STATIC_CREATE 199
#define K_SYSCALL_Z_SYS_MUTEX_KERNEL_LOCK 200
#define K_SYSCALL_Z_SYS_MUTEX_KERNEL_UNLOCK 201
#define K_SYSCALL_Z_ZSOCK_GETADDRINFO_INTERNAL 202
#define K_SYSCALL_BAD 203
#define K_SYSCALL_LIMIT 204


/* Following syscalls are not used in image */
#define K_SYSCALL_ADC_CHANNEL_SETUP 205
#define K_SYSCALL_ADC_READ 206
#define K_SYSCALL_ADC_READ_ASYNC 207
#define K_SYSCALL_ATOMIC_ADD 208
#define K_SYSCALL_ATOMIC_AND 209
#define K_SYSCALL_ATOMIC_CAS 210
#define K_SYSCALL_ATOMIC_NAND 211
#define K_SYSCALL_ATOMIC_OR 212
#define K_SYSCALL_ATOMIC_PTR_CAS 213
#define K_SYSCALL_ATOMIC_PTR_SET 214
#define K_SYSCALL_ATOMIC_SET 215
#define K_SYSCALL_ATOMIC_SUB 216
#define K_SYSCALL_ATOMIC_XOR 217
#define K_SYSCALL_AUXDISPLAY_BACKLIGHT_GET 218
#define K_SYSCALL_AUXDISPLAY_BACKLIGHT_SET 219
#define K_SYSCALL_AUXDISPLAY_BRIGHTNESS_GET 220
#define K_SYSCALL_AUXDISPLAY_BRIGHTNESS_SET 221
#define K_SYSCALL_AUXDISPLAY_CAPABILITIES_GET 222
#define K_SYSCALL_AUXDISPLAY_CLEAR 223
#define K_SYSCALL_AUXDISPLAY_CURSOR_POSITION_GET 224
#define K_SYSCALL_AUXDISPLAY_CURSOR_POSITION_SET 225
#define K_SYSCALL_AUXDISPLAY_CURSOR_SET_ENABLED 226
#define K_SYSCALL_AUXDISPLAY_CURSOR_SHIFT_SET 227
#define K_SYSCALL_AUXDISPLAY_CUSTOM_CHARACTER_SET 228
#define K_SYSCALL_AUXDISPLAY_CUSTOM_COMMAND 229
#define K_SYSCALL_AUXDISPLAY_DISPLAY_OFF 230
#define K_SYSCALL_AUXDISPLAY_DISPLAY_ON 231
#define K_SYSCALL_AUXDISPLAY_DISPLAY_POSITION_GET 232
#define K_SYSCALL_AUXDISPLAY_DISPLAY_POSITION_SET 233
#define K_SYSCALL_AUXDISPLAY_IS_BUSY 234
#define K_SYSCALL_AUXDISPLAY_POSITION_BLINKING_SET_ENABLED 235
#define K_SYSCALL_AUXDISPLAY_WRITE 236
#define K_SYSCALL_BBRAM_CHECK_INVALID 237
#define K_SYSCALL_BBRAM_CHECK_POWER 238
#define K_SYSCALL_BBRAM_CHECK_STANDBY_POWER 239
#define K_SYSCALL_BBRAM_GET_SIZE 240
#define K_SYSCALL_BBRAM_READ 241
#define K_SYSCALL_BBRAM_WRITE 242
#define K_SYSCALL_BC12_SET_RESULT_CB 243
#define K_SYSCALL_BC12_SET_ROLE 244
#define K_SYSCALL_CAN_ADD_RX_FILTER_MSGQ 245
#define K_SYSCALL_CAN_CALC_TIMING 246
#define K_SYSCALL_CAN_CALC_TIMING_DATA 247
#define K_SYSCALL_CAN_GET_BITRATE_MAX 248
#define K_SYSCALL_CAN_GET_BITRATE_MIN 249
#define K_SYSCALL_CAN_GET_CAPABILITIES 250
#define K_SYSCALL_CAN_GET_CORE_CLOCK 251
#define K_SYSCALL_CAN_GET_MAX_FILTERS 252
#define K_SYSCALL_CAN_GET_MODE 253
#define K_SYSCALL_CAN_GET_STATE 254
#define K_SYSCALL_CAN_GET_TIMING_DATA_MAX 255
#define K_SYSCALL_CAN_GET_TIMING_DATA_MIN 256
#define K_SYSCALL_CAN_GET_TIMING_MAX 257
#define K_SYSCALL_CAN_GET_TIMING_MIN 258
#define K_SYSCALL_CAN_GET_TRANSCEIVER 259
#define K_SYSCALL_CAN_RECOVER 260
#define K_SYSCALL_CAN_REMOVE_RX_FILTER 261
#define K_SYSCALL_CAN_SEND 262
#define K_SYSCALL_CAN_SET_BITRATE 263
#define K_SYSCALL_CAN_SET_BITRATE_DATA 264
#define K_SYSCALL_CAN_SET_MODE 265
#define K_SYSCALL_CAN_SET_TIMING 266
#define K_SYSCALL_CAN_SET_TIMING_DATA 267
#define K_SYSCALL_CAN_START 268
#define K_SYSCALL_CAN_STATS_GET_ACK_ERRORS 269
#define K_SYSCALL_CAN_STATS_GET_BIT0_ERRORS 270
#define K_SYSCALL_CAN_STATS_GET_BIT1_ERRORS 271
#define K_SYSCALL_CAN_STATS_GET_BIT_ERRORS 272
#define K_SYSCALL_CAN_STATS_GET_CRC_ERRORS 273
#define K_SYSCALL_CAN_STATS_GET_FORM_ERRORS 274
#define K_SYSCALL_CAN_STATS_GET_RX_OVERRUNS 275
#define K_SYSCALL_CAN_STATS_GET_STUFF_ERRORS 276
#define K_SYSCALL_CAN_STOP 277
#define K_SYSCALL_CHARGER_CHARGE_ENABLE 278
#define K_SYSCALL_CHARGER_GET_PROP 279
#define K_SYSCALL_CHARGER_SET_PROP 280
#define K_SYSCALL_COMPARATOR_GET_OUTPUT 281
#define K_SYSCALL_COMPARATOR_SET_TRIGGER 282
#define K_SYSCALL_COMPARATOR_TRIGGER_IS_PENDING 283
#define K_SYSCALL_COUNTER_CANCEL_CHANNEL_ALARM 284
#define K_SYSCALL_COUNTER_GET_FREQUENCY 285
#define K_SYSCALL_COUNTER_GET_GUARD_PERIOD 286
#define K_SYSCALL_COUNTER_GET_MAX_TOP_VALUE 287
#define K_SYSCALL_COUNTER_GET_NUM_OF_CHANNELS 288
#define K_SYSCALL_COUNTER_GET_PENDING_INT 289
#define K_SYSCALL_COUNTER_GET_TOP_VALUE 290
#define K_SYSCALL_COUNTER_GET_VALUE 291
#define K_SYSCALL_COUNTER_GET_VALUE_64 292
#define K_SYSCALL_COUNTER_IS_COUNTING_UP 293
#define K_SYSCALL_COUNTER_SET_CHANNEL_ALARM 294
#define K_SYSCALL_COUNTER_SET_GUARD_PERIOD 295
#define K_SYSCALL_COUNTER_SET_TOP_VALUE 296
#define K_SYSCALL_COUNTER_START 297
#define K_SYSCALL_COUNTER_STOP 298
#define K_SYSCALL_COUNTER_TICKS_TO_US 299
#define K_SYSCALL_COUNTER_US_TO_TICKS 300
#define K_SYSCALL_DAC_CHANNEL_SETUP 301
#define K_SYSCALL_DAC_WRITE_VALUE 302
#define K_SYSCALL_DEVMUX_SELECT_GET 303
#define K_SYSCALL_DEVMUX_SELECT_SET 304
#define K_SYSCALL_DMA_CHAN_FILTER 305
#define K_SYSCALL_DMA_RELEASE_CHANNEL 306
#define K_SYSCALL_DMA_REQUEST_CHANNEL 307
#define K_SYSCALL_DMA_RESUME 308
#define K_SYSCALL_DMA_START 309
#define K_SYSCALL_DMA_STOP 310
#define K_SYSCALL_DMA_SUSPEND 311
#define K_SYSCALL_EEPROM_GET_SIZE 312
#define K_SYSCALL_EEPROM_READ 313
#define K_SYSCALL_EEPROM_WRITE 314
#define K_SYSCALL_EMUL_FUEL_GAUGE_IS_BATTERY_CUTOFF 315
#define K_SYSCALL_EMUL_FUEL_GAUGE_SET_BATTERY_CHARGING 316
#define K_SYSCALL_ESPI_CONFIG 317
#define K_SYSCALL_ESPI_FLASH_ERASE 318
#define K_SYSCALL_ESPI_GET_CHANNEL_STATUS 319
#define K_SYSCALL_ESPI_READ_FLASH 320
#define K_SYSCALL_ESPI_READ_LPC_REQUEST 321
#define K_SYSCALL_ESPI_READ_REQUEST 322
#define K_SYSCALL_ESPI_RECEIVE_OOB 323
#define K_SYSCALL_ESPI_RECEIVE_VWIRE 324
#define K_SYSCALL_ESPI_SAF_ACTIVATE 325
#define K_SYSCALL_ESPI_SAF_CONFIG 326
#define K_SYSCALL_ESPI_SAF_FLASH_ERASE 327
#define K_SYSCALL_ESPI_SAF_FLASH_READ 328
#define K_SYSCALL_ESPI_SAF_FLASH_UNSUCCESS 329
#define K_SYSCALL_ESPI_SAF_FLASH_WRITE 330
#define K_SYSCALL_ESPI_SAF_GET_CHANNEL_STATUS 331
#define K_SYSCALL_ESPI_SAF_SET_PROTECTION_REGIONS 332
#define K_SYSCALL_ESPI_SEND_OOB 333
#define K_SYSCALL_ESPI_SEND_VWIRE 334
#define K_SYSCALL_ESPI_WRITE_FLASH 335
#define K_SYSCALL_ESPI_WRITE_LPC_REQUEST 336
#define K_SYSCALL_ESPI_WRITE_REQUEST 337
#define K_SYSCALL_FLASH_COPY 338
#define K_SYSCALL_FLASH_ERASE 339
#define K_SYSCALL_FLASH_EX_OP 340
#define K_SYSCALL_FLASH_FILL 341
#define K_SYSCALL_FLASH_FLATTEN 342
#define K_SYSCALL_FLASH_GET_PAGE_COUNT 343
#define K_SYSCALL_FLASH_GET_PAGE_INFO_BY_IDX 344
#define K_SYSCALL_FLASH_GET_PAGE_INFO_BY_OFFS 345
#define K_SYSCALL_FLASH_GET_PARAMETERS 346
#define K_SYSCALL_FLASH_GET_SIZE 347
#define K_SYSCALL_FLASH_GET_WRITE_BLOCK_SIZE 348
#define K_SYSCALL_FLASH_READ 349
#define K_SYSCALL_FLASH_READ_JEDEC_ID 350
#define K_SYSCALL_FLASH_SFDP_READ 351
#define K_SYSCALL_FLASH_SIMULATOR_GET_MEMORY 352
#define K_SYSCALL_FLASH_WRITE 353
#define K_SYSCALL_FUEL_GAUGE_BATTERY_CUTOFF 354
#define K_SYSCALL_FUEL_GAUGE_GET_BUFFER_PROP 355
#define K_SYSCALL_FUEL_GAUGE_GET_PROP 356
#define K_SYSCALL_FUEL_GAUGE_GET_PROPS 357
#define K_SYSCALL_FUEL_GAUGE_SET_PROP 358
#define K_SYSCALL_FUEL_GAUGE_SET_PROPS 359
#define K_SYSCALL_GNSS_GET_ENABLED_SYSTEMS 360
#define K_SYSCALL_GNSS_GET_FIX_RATE 361
#define K_SYSCALL_GNSS_GET_LATEST_TIMEPULSE 362
#define K_SYSCALL_GNSS_GET_NAVIGATION_MODE 363
#define K_SYSCALL_GNSS_GET_SUPPORTED_SYSTEMS 364
#define K_SYSCALL_GNSS_SET_ENABLED_SYSTEMS 365
#define K_SYSCALL_GNSS_SET_FIX_RATE 366
#define K_SYSCALL_GNSS_SET_NAVIGATION_MODE 367
#define K_SYSCALL_HAPTICS_START_OUTPUT 368
#define K_SYSCALL_HAPTICS_STOP_OUTPUT 369
#define K_SYSCALL_HWINFO_CLEAR_RESET_CAUSE 370
#define K_SYSCALL_HWINFO_GET_DEVICE_EUI64 371
#define K_SYSCALL_HWINFO_GET_DEVICE_ID 372
#define K_SYSCALL_HWINFO_GET_RESET_CAUSE 373
#define K_SYSCALL_HWINFO_GET_SUPPORTED_RESET_CAUSE 374
#define K_SYSCALL_HWSPINLOCK_GET_MAX_ID 375
#define K_SYSCALL_HWSPINLOCK_LOCK 376
#define K_SYSCALL_HWSPINLOCK_TRYLOCK 377
#define K_SYSCALL_HWSPINLOCK_UNLOCK 378
#define K_SYSCALL_I2S_BUF_READ 379
#define K_SYSCALL_I2S_BUF_WRITE 380
#define K_SYSCALL_I2S_CONFIGURE 381
#define K_SYSCALL_I2S_TRIGGER 382
#define K_SYSCALL_I3C_DO_CCC 383
#define K_SYSCALL_I3C_TRANSFER 384
#define K_SYSCALL_IPM_COMPLETE 385
#define K_SYSCALL_IPM_MAX_DATA_SIZE_GET 386
#define K_SYSCALL_IPM_MAX_ID_VAL_GET 387
#define K_SYSCALL_IPM_SEND 388
#define K_SYSCALL_IPM_SET_ENABLED 389
#define K_SYSCALL_IVSHMEM_ENABLE_INTERRUPTS 390
#define K_SYSCALL_IVSHMEM_GET_ID 391
#define K_SYSCALL_IVSHMEM_GET_MAX_PEERS 392
#define K_SYSCALL_IVSHMEM_GET_MEM 393
#define K_SYSCALL_IVSHMEM_GET_OUTPUT_MEM_SECTION 394
#define K_SYSCALL_IVSHMEM_GET_PROTOCOL 395
#define K_SYSCALL_IVSHMEM_GET_RW_MEM_SECTION 396
#define K_SYSCALL_IVSHMEM_GET_STATE 397
#define K_SYSCALL_IVSHMEM_GET_VECTORS 398
#define K_SYSCALL_IVSHMEM_INT_PEER 399
#define K_SYSCALL_IVSHMEM_REGISTER_HANDLER 400
#define K_SYSCALL_IVSHMEM_SET_STATE 401
#define K_SYSCALL_KSCAN_CONFIG 402
#define K_SYSCALL_KSCAN_DISABLE_CALLBACK 403
#define K_SYSCALL_KSCAN_ENABLE_CALLBACK 404
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_IN_GET 405
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_BACKING_STORE_PAGE_OUT_GET 406
#define K_SYSCALL_K_MEM_PAGING_HISTOGRAM_EVICTION_GET 407
#define K_SYSCALL_K_MEM_PAGING_STATS_GET 408
#define K_SYSCALL_K_MEM_PAGING_THREAD_STATS_GET 409
#define K_SYSCALL_LED_BLINK 410
#define K_SYSCALL_LED_GET_INFO 411
#define K_SYSCALL_LED_OFF 412
#define K_SYSCALL_LED_ON 413
#define K_SYSCALL_LED_SET_BRIGHTNESS 414
#define K_SYSCALL_LED_SET_CHANNEL 415
#define K_SYSCALL_LED_SET_COLOR 416
#define K_SYSCALL_LED_WRITE_CHANNELS 417
#define K_SYSCALL_LLEXT_GET_FN_TABLE 418
#define K_SYSCALL_MAXIM_DS3231_GET_SYNCPOINT 419
#define K_SYSCALL_MAXIM_DS3231_REQ_SYNCPOINT 420
#define K_SYSCALL_MBOX_MAX_CHANNELS_GET 421
#define K_SYSCALL_MBOX_MTU_GET 422
#define K_SYSCALL_MBOX_SEND 423
#define K_SYSCALL_MBOX_SET_ENABLED 424
#define K_SYSCALL_MDIO_BUS_DISABLE 425
#define K_SYSCALL_MDIO_BUS_ENABLE 426
#define K_SYSCALL_MDIO_READ 427
#define K_SYSCALL_MDIO_READ_C45 428
#define K_SYSCALL_MDIO_WRITE 429
#define K_SYSCALL_MDIO_WRITE_C45 430
#define K_SYSCALL_MSPI_CONFIG 431
#define K_SYSCALL_MSPI_DEV_CONFIG 432
#define K_SYSCALL_MSPI_GET_CHANNEL_STATUS 433
#define K_SYSCALL_MSPI_SCRAMBLE_CONFIG 434
#define K_SYSCALL_MSPI_TIMING_CONFIG 435
#define K_SYSCALL_MSPI_TRANSCEIVE 436
#define K_SYSCALL_MSPI_XIP_CONFIG 437
#define K_SYSCALL_NET_SOCKET_SERVICE_REGISTER 438
#define K_SYSCALL_NRF_QSPI_NOR_XIP_ENABLE 439
#define K_SYSCALL_PECI_CONFIG 440
#define K_SYSCALL_PECI_DISABLE 441
#define K_SYSCALL_PECI_ENABLE 442
#define K_SYSCALL_PECI_TRANSFER 443
#define K_SYSCALL_PS2_CONFIG 444
#define K_SYSCALL_PS2_DISABLE_CALLBACK 445
#define K_SYSCALL_PS2_ENABLE_CALLBACK 446
#define K_SYSCALL_PS2_READ 447
#define K_SYSCALL_PS2_WRITE 448
#define K_SYSCALL_PTP_CLOCK_GET 449
#define K_SYSCALL_PWM_CAPTURE_CYCLES 450
#define K_SYSCALL_PWM_DISABLE_CAPTURE 451
#define K_SYSCALL_PWM_ENABLE_CAPTURE 452
#define K_SYSCALL_PWM_GET_CYCLES_PER_SEC 453
#define K_SYSCALL_PWM_SET_CYCLES 454
#define K_SYSCALL_RESET_LINE_ASSERT 455
#define K_SYSCALL_RESET_LINE_DEASSERT 456
#define K_SYSCALL_RESET_LINE_TOGGLE 457
#define K_SYSCALL_RESET_STATUS 458
#define K_SYSCALL_RETAINED_MEM_CLEAR 459
#define K_SYSCALL_RETAINED_MEM_READ 460
#define K_SYSCALL_RETAINED_MEM_SIZE 461
#define K_SYSCALL_RETAINED_MEM_WRITE 462
#define K_SYSCALL_RTC_ALARM_GET_SUPPORTED_FIELDS 463
#define K_SYSCALL_RTC_ALARM_GET_TIME 464
#define K_SYSCALL_RTC_ALARM_IS_PENDING 465
#define K_SYSCALL_RTC_ALARM_SET_CALLBACK 466
#define K_SYSCALL_RTC_ALARM_SET_TIME 467
#define K_SYSCALL_RTC_GET_CALIBRATION 468
#define K_SYSCALL_RTC_GET_TIME 469
#define K_SYSCALL_RTC_SET_CALIBRATION 470
#define K_SYSCALL_RTC_SET_TIME 471
#define K_SYSCALL_RTC_UPDATE_SET_CALLBACK 472
#define K_SYSCALL_RTIO_CQE_COPY_OUT 473
#define K_SYSCALL_RTIO_CQE_GET_MEMPOOL_BUFFER 474
#define K_SYSCALL_RTIO_RELEASE_BUFFER 475
#define K_SYSCALL_RTIO_SQE_CANCEL 476
#define K_SYSCALL_RTIO_SQE_COPY_IN_GET_HANDLES 477
#define K_SYSCALL_RTIO_SUBMIT 478
#define K_SYSCALL_SDHC_CARD_BUSY 479
#define K_SYSCALL_SDHC_CARD_PRESENT 480
#define K_SYSCALL_SDHC_DISABLE_INTERRUPT 481
#define K_SYSCALL_SDHC_ENABLE_INTERRUPT 482
#define K_SYSCALL_SDHC_EXECUTE_TUNING 483
#define K_SYSCALL_SDHC_GET_HOST_PROPS 484
#define K_SYSCALL_SDHC_HW_RESET 485
#define K_SYSCALL_SDHC_REQUEST 486
#define K_SYSCALL_SDHC_SET_IO 487
#define K_SYSCALL_SENSOR_ATTR_GET 488
#define K_SYSCALL_SENSOR_ATTR_SET 489
#define K_SYSCALL_SENSOR_CHANNEL_GET 490
#define K_SYSCALL_SENSOR_GET_DECODER 491
#define K_SYSCALL_SENSOR_RECONFIGURE_READ_IODEV 492
#define K_SYSCALL_SENSOR_SAMPLE_FETCH 493
#define K_SYSCALL_SENSOR_SAMPLE_FETCH_CHAN 494
#define K_SYSCALL_SIP_SUPERVISORY_CALL 495
#define K_SYSCALL_SIP_SVC_PLAT_ASYNC_RES_REQ 496
#define K_SYSCALL_SIP_SVC_PLAT_ASYNC_RES_RES 497
#define K_SYSCALL_SIP_SVC_PLAT_FORMAT_TRANS_ID 498
#define K_SYSCALL_SIP_SVC_PLAT_FREE_ASYNC_MEMORY 499
#define K_SYSCALL_SIP_SVC_PLAT_FUNC_ID_VALID 500
#define K_SYSCALL_SIP_SVC_PLAT_GET_ERROR_CODE 501
#define K_SYSCALL_SIP_SVC_PLAT_GET_TRANS_IDX 502
#define K_SYSCALL_SIP_SVC_PLAT_UPDATE_TRANS_ID 503
#define K_SYSCALL_SMBUS_BLOCK_PCALL 504
#define K_SYSCALL_SMBUS_BLOCK_READ 505
#define K_SYSCALL_SMBUS_BLOCK_WRITE 506
#define K_SYSCALL_SMBUS_BYTE_DATA_READ 507
#define K_SYSCALL_SMBUS_BYTE_DATA_WRITE 508
#define K_SYSCALL_SMBUS_BYTE_READ 509
#define K_SYSCALL_SMBUS_BYTE_WRITE 510
#define K_SYSCALL_SMBUS_CONFIGURE 511
#define K_SYSCALL_SMBUS_GET_CONFIG 512
#define K_SYSCALL_SMBUS_HOST_NOTIFY_REMOVE_CB 513
#define K_SYSCALL_SMBUS_PCALL 514
#define K_SYSCALL_SMBUS_QUICK 515
#define K_SYSCALL_SMBUS_SMBALERT_REMOVE_CB 516
#define K_SYSCALL_SMBUS_WORD_DATA_READ 517
#define K_SYSCALL_SMBUS_WORD_DATA_WRITE 518
#define K_SYSCALL_STEPPER_ENABLE 519
#define K_SYSCALL_STEPPER_GET_ACTUAL_POSITION 520
#define K_SYSCALL_STEPPER_GET_MICRO_STEP_RES 521
#define K_SYSCALL_STEPPER_IS_MOVING 522
#define K_SYSCALL_STEPPER_MOVE_BY 523
#define K_SYSCALL_STEPPER_MOVE_TO 524
#define K_SYSCALL_STEPPER_RUN 525
#define K_SYSCALL_STEPPER_SET_EVENT_CALLBACK 526
#define K_SYSCALL_STEPPER_SET_MICROSTEP_INTERVAL 527
#define K_SYSCALL_STEPPER_SET_MICRO_STEP_RES 528
#define K_SYSCALL_STEPPER_SET_REFERENCE_POSITION 529
#define K_SYSCALL_SYSCON_GET_BASE 530
#define K_SYSCALL_SYSCON_GET_SIZE 531
#define K_SYSCALL_SYSCON_READ_REG 532
#define K_SYSCALL_SYSCON_WRITE_REG 533
#define K_SYSCALL_SYS_CACHE_DATA_FLUSH_AND_INVD_RANGE 534
#define K_SYSCALL_SYS_CACHE_DATA_FLUSH_RANGE 535
#define K_SYSCALL_SYS_CACHE_DATA_INVD_RANGE 536
#define K_SYSCALL_TEE_CANCEL 537
#define K_SYSCALL_TEE_CLOSE_SESSION 538
#define K_SYSCALL_TEE_GET_VERSION 539
#define K_SYSCALL_TEE_INVOKE_FUNC 540
#define K_SYSCALL_TEE_OPEN_SESSION 541
#define K_SYSCALL_TEE_SHM_ALLOC 542
#define K_SYSCALL_TEE_SHM_FREE 543
#define K_SYSCALL_TEE_SHM_REGISTER 544
#define K_SYSCALL_TEE_SHM_UNREGISTER 545
#define K_SYSCALL_TEE_SUPPL_RECV 546
#define K_SYSCALL_TEE_SUPPL_SEND 547
#define K_SYSCALL_TGPIO_PIN_CONFIG_EXT_TIMESTAMP 548
#define K_SYSCALL_TGPIO_PIN_DISABLE 549
#define K_SYSCALL_TGPIO_PIN_PERIODIC_OUTPUT 550
#define K_SYSCALL_TGPIO_PIN_READ_TS_EC 551
#define K_SYSCALL_TGPIO_PORT_GET_CYCLES_PER_SECOND 552
#define K_SYSCALL_TGPIO_PORT_GET_TIME 553
#define K_SYSCALL_UPDATEHUB_AUTOHANDLER 554
#define K_SYSCALL_UPDATEHUB_CONFIRM 555
#define K_SYSCALL_UPDATEHUB_PROBE 556
#define K_SYSCALL_UPDATEHUB_REBOOT 557
#define K_SYSCALL_UPDATEHUB_UPDATE 558
#define K_SYSCALL_USER_FAULT 559
#define K_SYSCALL_W1_CHANGE_BUS_LOCK 560
#define K_SYSCALL_W1_CONFIGURE 561
#define K_SYSCALL_W1_GET_SLAVE_COUNT 562
#define K_SYSCALL_W1_READ_BIT 563
#define K_SYSCALL_W1_READ_BLOCK 564
#define K_SYSCALL_W1_READ_BYTE 565
#define K_SYSCALL_W1_RESET_BUS 566
#define K_SYSCALL_W1_SEARCH_BUS 567
#define K_SYSCALL_W1_WRITE_BIT 568
#define K_SYSCALL_W1_WRITE_BLOCK 569
#define K_SYSCALL_W1_WRITE_BYTE 570
#define K_SYSCALL_WDT_DISABLE 571
#define K_SYSCALL_WDT_FEED 572
#define K_SYSCALL_WDT_SETUP 573


#ifndef _ASMLANGUAGE

#include <stdarg.h>
#include <stdint.h>

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_SYSCALL_LIST_H */
