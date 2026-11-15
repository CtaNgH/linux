/* chopin: vendor-era CMDQ prototypes, so mediatek_v2 sees the signatures it was written against */
#ifndef __CHOPIN_CMDQ_SHIM_H
#define __CHOPIN_CMDQ_SHIM_H
#include <linux/types.h>

int cmdq_pkt_flush(struct cmdq_pkt *pkt);
int cmdq_pkt_wait_no_clear(struct cmdq_pkt *pkt, u16 event);
int cmdq_pkt_wfe(struct cmdq_pkt *pkt, u16 event);
s32 cmdq_pkt_assign_command(struct cmdq_pkt *pkt, u16 reg_idx, u32 value);
s32 cmdq_pkt_clear_event(struct cmdq_pkt *pkt, u16 event);
s32 cmdq_pkt_cond_jump_abs(struct cmdq_pkt *pkt,
	u16 addr_reg_idx,
	struct cmdq_operand *left_operand,
	struct cmdq_operand *right_operand,
	enum CMDQ_CONDITION_ENUM condition_operator);
s32 cmdq_pkt_finalize_loop(struct cmdq_pkt *pkt);
s32 cmdq_pkt_flush_async(struct cmdq_pkt *pkt,
	cmdq_async_flush_cb cb, void *data);
s32 cmdq_pkt_flush_threaded(struct cmdq_pkt *pkt,
	cmdq_async_flush_cb cb, void *data);
s32 cmdq_pkt_logic_command(struct cmdq_pkt *pkt, enum CMDQ_LOGIC_ENUM s_op,
	u16 result_reg_idx,
	struct cmdq_operand *left_operand,
	struct cmdq_operand *right_operand);
s32 cmdq_pkt_mem_move(struct cmdq_pkt *pkt, void *clt_base,
	dma_addr_t src_addr, dma_addr_t dst_addr, u16 swap_reg_idx);
s32 cmdq_pkt_poll_reg(struct cmdq_pkt *pkt, u32 value, u8 subsys,
	u16 offset, u32 mask);
s32 cmdq_pkt_poll_timeout(struct cmdq_pkt *pkt, u32 value, u8 subsys,
	phys_addr_t addr, u32 mask, u16 count, u16 reg_gpr);
s32 cmdq_pkt_read(struct cmdq_pkt *pkt, void *clt_base,
	dma_addr_t src_addr, u16 dst_reg_idx);
s32 cmdq_pkt_set_event(struct cmdq_pkt *pkt, u16 event);
s32 cmdq_pkt_write(struct cmdq_pkt *pkt, void *clt_base, ...);
s32 cmdq_pkt_write_reg_addr(struct cmdq_pkt *pkt, dma_addr_t addr,
	u16 src_reg_idx, u32 mask);
struct cmdq_pkt *cmdq_pkt_create(struct cmdq_client *client);
void cmdq_pkt_destroy(struct cmdq_pkt *pkt);

#endif
