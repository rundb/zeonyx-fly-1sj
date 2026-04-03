#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

/* Example custom shell command */
static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	shell_print(sh, "Hello from STM32L072CZ!");
	return 0;
}

SHELL_CMD_REGISTER(hello, NULL, "Print a greeting", cmd_hello);

int main(void)
{
	/* Shell starts automatically via Zephyr kernel on USART2 (PA2/PA3) */
	return 0;
}
