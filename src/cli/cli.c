#include <stdlib.h>
#include "embedded_cli.h"

static EmbeddedCli *g_cli = NULL;
static void (*g_send_char_func)(char c) = NULL;

static void writeChar(EmbeddedCli *cli, char c)
{
    (void)(cli);
    g_send_char_func(c);
}


static void on_recovery(EmbeddedCli *cli, char *args, void *context) {
    embeddedCliPrint(cli, "Running recovery..");
}

static void on_reset(EmbeddedCli *cli, char *args, void *context) {
    embeddedCliPrint(cli, "Running reset..");
}


int cli_init( void (*send_char)(char) )
{
    EmbeddedCliConfig *config = embeddedCliDefaultConfig();
    EmbeddedCli *cli = embeddedCliNew(config);

    g_send_char_func = send_char;
    cli->writeChar = writeChar;
    CliCommandBinding cmd_recovery = {
        "recovery", "Put Alpha in recovery mode", false, NULL, on_recovery 
    };
    
    CliCommandBinding cmd_reset = {
        "reset", "Reboot Alpha", true, NULL, on_reset 
    };

    embeddedCliAddBinding(cli, cmd_recovery);
    
    embeddedCliAddBinding(cli, cmd_reset);

    g_cli = cli;

    return 0;
}

int cli_putn(const char *c, unsigned n) 
{
    for (unsigned i = 0; i < n; i++) {
        embeddedCliReceiveChar(g_cli, c[i]);
    }
    return 0;
}

int cli_update(void) 
{
    embeddedCliProcess(g_cli);
    return 0;
}