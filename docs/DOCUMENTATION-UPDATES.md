# Documentation updates / 展示文档更新

## 2026-10-27 — Organizer-requested bilingual documentation

The organizer requested bilingual **Project Overview, Demo Video, Architecture, Integration Standards**, with English displayed by default. / 按组委会通知补齐四项中英文内容，默认展示英文。

| Requirement / 要求 | English | 简体中文 |
|---|---|---|
| Project overview / 项目介绍 | [README](../README.md#project-overview) | [README.zh-CN](../README.zh-CN.md#项目介绍project-overview) |
| Demo video / 演示视频 | [README](../README.md#demo-video) | [README.zh-CN](../README.zh-CN.md#视频介绍demo-video) |
| Architecture / 架构介绍 | [Architecture](ARCHITECTURE.en.md) | [架构说明](ARCHITECTURE.md) |
| Integration standards / 集成标准 | [Integration](INTEGRATION.md) | [集成规范](INTEGRATION.zh-CN.md) |

This update changes publication documentation and refreshes its public-file hash inventory. It does **not** change the submitted application's Python files, task prompt, map data, pinned implementation manifest, original `competition-v14` tag or existing release assets. The `provenance/public-files.json` in each Git revision describes that revision's public file set; the original tag retains the original inventory.

此次更新只完善公开说明并刷新公开文件哈希清单，不修改原参赛Python实现、提示词、地图、执行入口冻结清单、原参赛标签或已有Release附件。每个Git版本的公开清单对应自身文件状态，原标签仍保留首次公开清单。

Some provenance files contain `git_tag_created=false` or `github_published=false`: these are historical **pre-publication preparation snapshots**, not current repository status. The actual published tag, release and CI are linked from the READMEs. They remain unchanged to preserve provenance.

部分来源记录中的 `git_tag_created=false` 或 `github_published=false`为**发布前快照**，不代表当前状态；保留原记录，通过README链接查看实际发布页、标签与CI。

Original release commit / 首次公开提交：`198ce2a2baaca1169aae5d6d1d2b27f6518e9c41`
Frozen tag / 固定标签：[competition-v14](https://github.com/HoseaZou/EmbodiedGuard/tree/competition-v14)
