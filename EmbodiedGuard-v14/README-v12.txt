EmbodiedGuard v12：六工具语义导航候选，基于冻结v9并参考v11反馈投影

显式 --response-mode compact --interface-profile focused。
Agent公开工具仅 observe、navigate、reobserve、wait、stop、verify。旧profile仍保留原工具。
navigate必传route_mode：plan明确调用原规划并执行；resume只沿当前同目标/同路线句柄执行，未匹配时返回失败，不静默重规划。
原规划、执行、停稳、实际到达巡检仍在确定性底层；不自动选择路线、失败恢复、下一目标或最终verify。
每次仅一个15秒窗口，非v10双窗口。所有原安全、传感器、任务权限、真实等待、截止300秒、80执行、120请求及3恢复限额保留。
完整原结果日志保留；未知和失败反馈保留；最后verify不额外压缩。不宣称新模型、实机/视觉准确率、固定加速或已提交比赛。
