/******************************************************************************
Copyright (C) 2021-2023 广州敏视数码科技有限公司版权所有.

文件名：main.cpp

日期: 2021-08-03

文件功能描述: 定义媒体接口访问demo

其他: // 其他内容说明

版本: v1.0.0(最新版本号)

*******************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <sys/socket.h>
#include <sys/fcntl.h>
#include <sys/un.h>

#include <pthread.h>
#include <ctype.h>
#include <fcntl.h>
#include <errno.h>
#include <pthread.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <signal.h>


#include "print.h"
#include "common.h"
#include "config.h"
#include "../../../../include/board.h"
#include "op.h"
#include "msg.h"
#include "alarm.h"
#include "unsocket.h"
#include "cJSON.h"
#include "mongoose.h"


static const char *s_listen_on = "ws://0.0.0.0:8000";
static const char *s_web_root = ".";
static int g_get_warning = 0;

// This RESTful server implements the following endpoints:
//   /websocket - upgrade to Websocket, and implement websocket echo server
//   /api/rest - respond with JSON string {"result": 123}
//   any other URI serves static files from s_web_root
static void fn(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (ev == MG_EV_OPEN) 
    {
        // c->is_hexdumping = 1;
    } 
    else if (ev == MG_EV_HTTP_MSG) 
    {
        struct mg_http_message *hm = (struct mg_http_message *) ev_data;
        if (mg_http_match_uri(hm, "/websocket")) 
        {
            // Upgrade to websocket. From now on, a connection is a full-duplex
            // Websocket connection, which will receive MG_EV_WS_MSG events.
            mg_ws_upgrade(c, hm, NULL);
        } 
        else if (mg_http_match_uri(hm, "/rest")) 
        {
            // Serve REST response
            mg_http_reply(c, 200, "", "{\"result\": %d}\n", 123);
        } 
        else 
        {
            // Serve static files
            struct mg_http_serve_opts opts = {.root_dir = s_web_root};
            mg_http_serve_dir(c, ev_data, &opts);
        }
    } 
    else if (ev == MG_EV_WS_MSG) 
    {
        // Got websocket frame. Received data is wm->data. Echo it back!
        struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
        if (0 == memcmp(wm->data.ptr, "get_warning", strlen("get_warning")))
        {
            printf("get_warning\n");
            g_get_warning = 1;
            c->label[0]='A';
        }
        else
        {
            mg_ws_send(c, wm->data.ptr, wm->data.len, WEBSOCKET_OP_TEXT);
        }
    }
    (void) fn_data;
}

void * websocket_body(void *pvArg)
{
    uint32 u32Cnt = 0;
    struct mg_mgr *pmgr = (struct mg_mgr *)pvArg;  // Event manager
    struct mg_connection *tmp_c;
    char szAlarmBuf[64];

    while(1)
    {
        sleep_ms(500);
        if (g_get_warning)
        {          
            for (tmp_c = pmgr->conns; tmp_c != NULL; tmp_c = tmp_c->next)
            {
                if (tmp_c->label[0] != 'A')
                    continue;               // Skip non-stream connections

                u32Cnt++;
                sprintf(szAlarmBuf, "Alarm Num: %d", u32Cnt);
                mg_ws_send(tmp_c, szAlarmBuf, strlen(szAlarmBuf), WEBSOCKET_OP_TEXT);
            }      
        }
    }

    return NULL;
}

int main(int argc, char **argv)
{
    sint32 s32Ret;
    pthread_t thread;
    struct mg_mgr mgr;  // Event manager
    
    mg_mgr_init(&mgr);  // Initialise event manager
    printf("Starting WS listener on %s/websocket\n", s_listen_on);
    mg_http_listen(&mgr, s_listen_on, fn, NULL);  // Create HTTP listener

    s32Ret = pthread_create(&thread, NULL, websocket_body, &mgr);
    if (0 != s32Ret)
    {
        printf("pthread_create failed. [err=%#x]\n", s32Ret);
        return -1;
    }
    
    for (;;) 
    {
        mg_mgr_poll(&mgr, 1000);             // Infinite event loop
    }
    mg_mgr_free(&mgr);
    return 0;
}

