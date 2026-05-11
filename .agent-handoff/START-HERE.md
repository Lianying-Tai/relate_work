# 交接入口

## 读取顺序

1. `STATE.yaml`
2. `CURRENT.md`
3. `TASKS.md`
4. `DECISIONS.md`
5. `SESSION-LOG.md`
6. `DAILY-LOG.md`
7. `HANDOFF-RULES.md`

## 恢复方式

在另一台电脑克隆仓库后，如果已安装 Python 3.8+ 和 PyYAML，可以运行：

```bash
python .agent-handoff/runtime/agent-handoff/scripts/handoff.py resume .
```

如果使用 Codex 的 `agent-handoff` skill，可让 agent 读取当前仓库的交接状态并继续工作。

