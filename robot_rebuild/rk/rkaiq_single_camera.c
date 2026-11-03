/* Single OV5647 AIQ owner. Motor/serial interfaces are never opened here.
 * C ABI: supplied rk_aiq_user_api2_sysctl.h; camera lifecycle follows the
 * supplied rkaiq_3A_server.cpp, restricted to the discovered sensor/ISP pair.
 * Build on RK: gcc -O2 -Wall -Wextra -o rkaiq-single-camera.bin this.c -ldl
 * Run with the matching isolated loader/libraries through rkaiq-wrapper.sh.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/videodev2.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define ISP_STREAM_START (V4L2_EVENT_PRIVATE_START + 1)
#define ISP_STREAM_STOP  (V4L2_EVENT_PRIVATE_START + 2)
static volatile sig_atomic_t ending;
static void terminate(int signal_number) { (void)signal_number; ending=1; }
static unsigned dimension(const char *name) {
    const char *text=getenv(name);char *end=NULL;
    if (!text || !*text)return 0;
    errno=0;unsigned long value=strtoul(text,&end,10);
    return errno || *end || value>8192 ? 0 : (unsigned)value;
}

int main(void) {
    setvbuf(stdout,NULL,_IOLBF,0);
    const char *sensor=getenv("ACG138_SENSOR");
    const char *device=getenv("ACG138_ISP_PARAMS");
    unsigned width=dimension("ACG138_WIDTH"),height=dimension("ACG138_HEIGHT");
    if (!((width==640 && height==480) || (width==1296 && height==972)
            || (width==1920 && height==1080) || (width==2592 && height==1944))) {
        fprintf(stderr,"Unsupported OV5647 source size\n");return 2;
    }
    if (!sensor || !*sensor || !device || strncmp(device,"/dev/video",10)) {
        fprintf(stderr,"Missing discovered OV5647 sensor/ISP environment\n");return 2;
    }
    void *lib=dlopen("/userdata/acg138-camera/lib/librkaiq.so",RTLD_NOW|RTLD_LOCAL);
    if (!lib) { fprintf(stderr,"AIQ dlopen: %s\n",dlerror());return 2; }
    void *(*init)(const char*,const char*,void*,void*)=dlsym(lib,"rk_aiq_uapi2_sysctl_init");
    void (*deinit)(void*)=dlsym(lib,"rk_aiq_uapi2_sysctl_deinit");
    void (*listen_stream)(void*,bool)=dlsym(lib,"rk_aiq_uapi2_sysctl_setListenStrmStatus");
    int (*prepare)(void*,unsigned int,unsigned int,int)=dlsym(lib,"rk_aiq_uapi2_sysctl_prepare");
    int (*start)(void*)=dlsym(lib,"rk_aiq_uapi2_sysctl_start");
    int (*stop)(void*,bool)=dlsym(lib,"rk_aiq_uapi2_sysctl_stop");
    if (!init || !deinit || !listen_stream || !prepare || !start || !stop) {
        fprintf(stderr,"Incomplete supplied AIQ API\n");return 2;
    }
    struct sigaction action={0};action.sa_handler=terminate;sigemptyset(&action.sa_mask);
    sigaction(SIGTERM,&action,NULL);sigaction(SIGINT,&action,NULL);
    int fd=open(device,O_RDWR|O_NONBLOCK|O_CLOEXEC);
    if (fd<0) { perror("open ISP params");return 2; }
    int result=1;bool running=false;void *ctx=NULL;
    struct v4l2_event_subscription sub={0};
    printf("Single camera: %s via %s\n",sensor,device);
    ctx=init(sensor,"/etc/iqfiles",NULL,NULL);
    if (!ctx) { fprintf(stderr,"AIQ init returned NULL\n");goto done; }
    listen_stream(ctx,false);
    if (prepare(ctx,width,height,0)!=0) { fprintf(stderr,"AIQ prepare failed\n");goto done; }
    sub.type=ISP_STREAM_START;
    if (ioctl(fd,VIDIOC_SUBSCRIBE_EVENT,&sub)<0) { perror("subscribe START");goto done; }
    sub.type=ISP_STREAM_STOP;
    if (ioctl(fd,VIDIOC_SUBSCRIBE_EVENT,&sub)<0) { perror("subscribe STOP");goto done; }
    printf("wait stream start event...\n");
    while (!ending) {
        /* Vendor params node reports POLLERR before stream-on. Dequeue
         * directly with O_NONBLOCK, keeping TERM handling bounded. */
        struct v4l2_event event={0};
        if (ioctl(fd,VIDIOC_DQEVENT,&event)<0) {
            /* V4L2 __v4l2_event_dequeue returns ENOENT for an empty
             * subscribed queue when opened nonblocking. */
            if(errno==EAGAIN || errno==ENOENT) { usleep(20000);continue; }
            if(errno==EINTR)continue;
            perror("dequeue ISP event");goto done;
        }
        if (event.type==ISP_STREAM_START && !running) {
            if (start(ctx)!=0) { fprintf(stderr,"AIQ start failed\n");goto done; }
            running=true;printf("AIQ stream started\n");
        } else if (event.type==ISP_STREAM_STOP && running) {
            int status=stop(ctx,false);running=false;
            if (status!=0) { fprintf(stderr,"AIQ stop failed\n");goto done; }
            printf("AIQ stream stopped; wait stream start event...\n");
        }
    }
    result=0;
done:
    if (running && ctx)stop(ctx,false);
    if (ctx)deinit(ctx);
    sub.type=ISP_STREAM_START;ioctl(fd,VIDIOC_UNSUBSCRIBE_EVENT,&sub);
    sub.type=ISP_STREAM_STOP;ioctl(fd,VIDIOC_UNSUBSCRIBE_EVENT,&sub);
    close(fd);dlclose(lib);return result;
}
