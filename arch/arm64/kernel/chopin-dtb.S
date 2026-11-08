/*
 * The board device tree, linked into the kernel image itself.
 *
 * setup_arch() replaces the FDT the bootloader handed over with this one.
 * That is not a stylistic choice: MTK's LK applies the dtbo partition on top of
 * the base tree it is given, and refusing/corrupting that step makes LK give up
 * before it ever starts the kernel.  So we hand LK a tree it is happy with
 * (the stock one) and swap in ours a moment later, inside the kernel.
 */
	.section .rodata, "a"
	.balign 8
	.global _binary_chopin_board_dtb_start
	.global _binary_chopin_board_dtb_end
_binary_chopin_board_dtb_start:
	.incbin "arch/arm64/kernel/chopin-board.dtb"
_binary_chopin_board_dtb_end:
