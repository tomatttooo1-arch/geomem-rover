/* Diagnostic-only malloc observer. No allocation interposition or ROS callbacks.
 * Socket commands run on a dedicated thread, never in an async signal handler.
 */
#define _GNU_SOURCE
#include <malloc.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>
#include <sys/mman.h>

#ifndef FPGA_CALIBRATION_NO_OBSERVER
static const char *directory;
static unsigned sequence;
static long long mono_ns(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec*1000000000LL+t.tv_nsec;
}
static int snapshot(char *path, size_t size) {
    snprintf(path,size,"%s/%06u.xml",directory,sequence++);
    FILE *f=fopen(path,"wx"); if (!f) return -1;
    int result=malloc_info(0,f); fclose(f); return result;
}
static void *serve(void *unused) {
    (void)unused;
    int server=socket(AF_UNIX,SOCK_SEQPACKET|SOCK_CLOEXEC,0);
    struct sockaddr_un address={.sun_family=AF_UNIX};
    if (snprintf(address.sun_path,sizeof(address.sun_path),"%s/control.sock",directory)>=(int)sizeof(address.sun_path)) return NULL;
    if (server<0 || bind(server,(struct sockaddr*)&address,sizeof(address)) || listen(server,4)) return NULL;
    for (;;) {
        int client=accept4(server,NULL,NULL,SOCK_CLOEXEC); if(client<0) continue;
        char command[32]={0}, before[512],after[512],reply[1300];
        ssize_t count=recv(client,command,sizeof(command)-1,0);
        if(count<=0 || (strcmp(command,"snapshot") && strcmp(command,"trim"))) {close(client);continue;}
        long long begin=mono_ns(); int a=snapshot(before,sizeof(before));
        int trimmed=strcmp(command,"trim") ? -1 : malloc_trim(0);
        int b=snapshot(after,sizeof(after));
        snprintf(reply,sizeof(reply),"{\"begin_ns\":%lld,\"end_ns\":%lld,\"before\":\"%s\",\"after\":\"%s\",\"trim_result\":%d,\"ok\":%s}",
                 begin,mono_ns(),before,after,trimmed,(a==0 && b==0)?"true":"false");
        send(client,reply,strlen(reply),MSG_NOSIGNAL); close(client);
    }
    return NULL;
}
__attribute__((constructor)) static void initialize(void) {
    const char *target=getenv("FPGA_ALLOCATOR_EXECUTABLE");
    if(target && *target) {
        char executable[4096];ssize_t length=readlink("/proc/self/exe",executable,sizeof(executable)-1);
        if(length<0)return;
        executable[length]=0;
        if(strcmp(executable,target))return;
    }
    directory=getenv("FPGA_ALLOCATOR_DIR");
    if(directory && *directory) {pthread_t thread; if(!pthread_create(&thread,NULL,serve,NULL)) pthread_detach(thread);}
}
#endif

#ifdef FPGA_CALIBRATION
static void *blocks[8192], *anchors[8192], *mapping;
__attribute__((noinline)) static void calibration_allocate(int mapped) {
    if(mapped) {mapping=mmap(NULL,64UL<<20,PROT_READ|PROT_WRITE,MAP_PRIVATE|MAP_ANONYMOUS,-1,0); if(mapping==MAP_FAILED) exit(4); memset(mapping,42,64UL<<20);return;}
    mallopt(M_MMAP_THRESHOLD,1024*1024*1024); mallopt(M_TRIM_THRESHOLD,-1);
    for(int i=0;i<8192;i++) {blocks[i]=malloc(8192);anchors[i]=malloc(64);if(!blocks[i]||!anchors[i])exit(4);memset(blocks[i],42,8192);memset(anchors[i],42,64);}
}
__attribute__((noinline)) static void calibration_release(int mapped) {
    if(mapped) {munmap(mapping,64UL<<20);return;}
    for(int i=0;i<8192;i++) {free(blocks[i]);blocks[i]=NULL;}
}
int main(int argc,char **argv) {
    int mapped=argc>1 && !strcmp(argv[1],"mmap");
    setvbuf(stdout,NULL,_IOLBF,0);printf("ready\n");
    for(int c;(c=getchar())!=EOF;) {
        if(c=='a'){calibration_allocate(mapped);printf("allocated\n");}
        if(c=='f'){calibration_release(mapped);printf("freed\n");}
        if(c=='q')break;
    }
    return 0;
}
#endif
