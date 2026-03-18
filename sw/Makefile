
test.elf:
	riscv64-linux-gnu-gcc -march=rv32imczicsr_zba_zbb_zbs -mabi=ilp32 \
	-ffreestanding -nostdlib -Wl,-T,linkerscript.ld,--noinhibit-exec \
	source/main.c source/uart.c source/point_cloud_manager.c \
	source/hw_ransac.c source/text_utils.c source/start.S \
	source/stdlib_utils.c source/sw_ransac.c \
	-Os -I include -o test.elf -g