# Local Jazzy / Ubuntu 24.04 compatibility

Enabled the existing pthread.h include in ldlidar_driver/include/logger/log_module.h. GCC otherwise cannot declare pthread_mutex_init/lock/unlock. Upstream commit is recorded in UPSTREAM.txt.
