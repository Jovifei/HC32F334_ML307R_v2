
## Workflow Orchestration

### 1. Plan Node Default

- Enter plan mode for ANY non-trivial task (3+ steps or architectural decisions)

- If something goes sideways, STOP and re-plan immediately - don't keep pushing

- Use plan mode for verification steps, not just building

- Write detailed specs upfront to reduce ambiguity

### 2. Subagent Strategy

- Use subagents liberally to keep main context window clean

- Offload research, exploration, and parallel analysis to subagents

- For complex problems, throw more compute at it via subagents

- One tack per subagent for focused execution

### 3. Self-Improvement Loop

- After ANY correction from the user: update `tasks/lessons.md` with the pattern

- Write rules for yourself that prevent the same mistake

- Ruthlessly iterate on these lessons until mistake rate drops

- Review lessons at session start for relevant project

### 4. Verification Before Done

- Never mark a task complete without proving it works

- Diff behavior between main and your changes when relevant

- Ask yourself: "Would a staff engineer approve this?"

- Run tests, check logs, demonstrate correctness

### 5. Demand Elegance (Balanced)

- For non-trivial changes: pause and ask "is there a more elegant way?"

- If a fix feels hacky: "Knowing everything I know now, implement the elegant solution"

- Skip this for simple, obvious fixes - don't over-engineer

- Challenge your own work before presenting it

### 6. Autonomous Bug Fixing

- When given a bug report: just fix it. Don't ask for hand-holding

- Point at logs, errors, failing tests - then resolve them

- Zero context switching required from the user

- Go fix failing CI tests without being told how

## Task Management

1. **Plan First**: Write plan to `tasks/todo.md` with checkable items

2. **Verify Plan**: Check in before starting implementation

3. **Track Progress**: Mark items complete as you go

4. **Explain Changes**: High-level summary at each step

5. **Document Results**: Add review section to `tasks/todo.md`

6. **Capture Lessons**: Update `tasks/lessons.md` after corrections

## Core Principles

- **Simplicity First**: Make every change as simple as possible. Impact minimal code.

- **No Laziness**: Find root causes. No temporary fixes. Senior developer standards.

- **Minimat Impact**: Changes should only touch what's necessary. Avoid introducing bugs.


### 7. Claude 指令配置

# 1.称呼规范

**每次回复时都必须称呼用户为"Jovi"**

### 2.进度可视化

所有任务执行时必须提供进度反馈，避免用户以为卡死:
**批量操作**:显示当前处理项，如[1/50]processingfile.py
**耗时任务**:定期输出进度，如 √ 已完成50%
**长时间任务**:使用TodoWrite工具跟踪，实时更新状态

# 3.禁止行为
长时间沉默不输出任何内容
只在最后给出结果，中间无反馈
批量操作时不显示当前处理项

# 4.文件下载路径（全局）
允许下载的路径：`E:\Claude_allow\Download`
所有下载操作必须在此目录下进行

# 5.AI工具安装目录（E盘分类）
AI工具统一安装在 `E:\AI_Tools\` 下，按类别分类：

```
E:\AI_Tools\
├── Claude\                   # Claude系列工具
│   └── ClaudeCode\           # Claude Code
│       ├── config\           # 配置文件
│       ├── data\             # 数据目录
│       ├── skills\           # Claude Skills
│       └── logs\             # 日志
├── Obsidian\                 # Obsidian笔记
│   ├── config\               # Obsidian配置
│   ├── data\                 # Obsidian主程序
│   ├── Data\                 # 笔记库（统一管理）
│   │   ├── notes-personal\   # 个人笔记vault
│   │   ├── notes-work\       # 工作笔记vault
│   │   └── notes-research\   # 研究笔记vault
│   └── logs\                 # Obsidian日志
├── Cursor\                   # Cursor编辑器
│   ├── config\               # Cursor配置
│   ├── data\                 # Cursor数据
│   ├── extensions\           # 扩展插件
│   └── logs\                 # 日志
├── VSCode\                   # VS Code（含AI扩展）
│   ├── config\               # VS Code配置
│   ├── data\                 # 数据目录
│   ├── extensions\           # 扩展
│   └── logs\                 # 日志
├── ChatGPT\                  # ChatGPT桌面端
├── Gemini\                   # Gemini工具
├── Copilot\                  # GitHub Copilot
└── Other\                    # 其他AI工具
    ├── StableDiffusion\      # 绘图AI
    └── LocalLLM\             # 本地大模型

E:\Claude_allow\               # 临时下载目录
└── Download\                 # 下载位置（允许全局下载）
```
