/*
 * glove_bridge.c — Runs inside the Docker container.
 * Captures `ros2 topic echo` output for both gloves and streams over TCP.
 *
 * Build (inside container):
 *   gcc -Wall -O2 -o glove_bridge glove_bridge.c -lpthread
 *
 * Each line sent to clients is prefixed: GLOVE0: or GLOVE1:
 * Message boundaries are "GLOVEX:---" lines.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

#define PORT        9090
#define MAX_CLIENTS 8
#define LINE_BUF    4096

static volatile sig_atomic_t running = 1;
static void on_signal(int sig) { (void)sig; running = 0; }

static int           client_fds[MAX_CLIENTS];
static int           num_clients = 0;
static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;

static void broadcast(const char *data, int len) {
    pthread_mutex_lock(&mu);
    for (int i = 0; i < num_clients; ) {
        ssize_t n = send(client_fds[i], data, len, MSG_NOSIGNAL);
        if (n < 0) {
            close(client_fds[i]);
            client_fds[i] = client_fds[--num_clients];
        } else {
            i++;
        }
    }
    pthread_mutex_unlock(&mu);
}

static void *accept_thread(void *arg) {
    int sfd = *(int *)arg;
    while (running) {
        int fd = accept(sfd, NULL, NULL);
        if (fd < 0) continue;
        int flag = 1;
        setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        pthread_mutex_lock(&mu);
        if (num_clients < MAX_CLIENTS)
            client_fds[num_clients++] = fd;
        else
            close(fd);
        pthread_mutex_unlock(&mu);
        fprintf(stderr, "[bridge] client connected (%d total)\n", num_clients);
    }
    return NULL;
}

struct topic_args {
    const char *topic;
    const char *prefix;
};

static void *topic_thread(void *arg) {
    struct topic_args *ta = arg;
    char cmd[512];
    char line[LINE_BUF];

    while (running) {
        snprintf(cmd, sizeof(cmd), "ros2 topic echo %s", ta->topic);

        FILE *fp = popen(cmd, "r");
        if (!fp) {
            perror("popen");
            sleep(1);
            continue;
        }

        fprintf(stderr, "[bridge] following topic %s\n", ta->topic);
        while (running && fgets(line, sizeof(line), fp)) {
            char out[LINE_BUF + 16];
            int len = snprintf(out, sizeof(out), "%s%s", ta->prefix, line);
            broadcast(out, len);
        }

        pclose(fp);
        if (running) {
            fprintf(stderr, "[bridge] topic reader for %s exited, retrying\n", ta->topic);
            sleep(1);
        }
    }
    return NULL;
}

int main(void) {
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);
    signal(SIGPIPE, SIG_IGN);

    int sfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sfd < 0) { perror("socket"); return 1; }

    int opt = 1;
    setsockopt(sfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(sfd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); return 1;
    }
    listen(sfd, 5);
    fprintf(stderr, "[bridge] listening on port %d\n", PORT);

    pthread_t tid;
    pthread_create(&tid, NULL, accept_thread, &sfd);

    static struct topic_args args[2] = {
        { "/manus_glove_0", "GLOVE0:" },
        { "/manus_glove_1", "GLOVE1:" },
    };
    pthread_t rtids[2];
    for (int i = 0; i < 2; i++)
        pthread_create(&rtids[i], NULL, topic_thread, &args[i]);

    for (int i = 0; i < 2; i++)
        pthread_join(rtids[i], NULL);

    close(sfd);
    return 0;
}
