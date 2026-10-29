# Issue tracker: GitHub

本仓库的任务与规格存放于 pawn-queen/px4_ros_ws 的 GitHub Issues。
使用 gh CLI；命令显式指定 --repo pawn-queen/px4_ros_ws。

## 常用操作

- 创建：gh issue create --repo pawn-queen/px4_ros_ws --title "标题" --body-file <文件>
- 读取：gh issue view <编号> --repo pawn-queen/px4_ros_ws --json number,title,body,labels,comments
- 列表：gh issue list --repo pawn-queen/px4_ros_ws --state open --json number,title,labels
- 评论：gh issue comment <编号> --repo pawn-queen/px4_ros_ws --body-file <文件>
- 标签：gh issue edit <编号> --repo pawn-queen/px4_ros_ws --add-label "<标签>"
- 关闭：gh issue close <编号> --repo pawn-queen/px4_ros_ws

多行正文写入临时文件，通过 --body-file 提交。
技能要求“发布到问题跟踪器”时创建 GitHub Issue；
要求“读取相关任务”时读取对应 Issue 及评论。

## Pull requests as a triage surface

PRs as a request surface: no.
