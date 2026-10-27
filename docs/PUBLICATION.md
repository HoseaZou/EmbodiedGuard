# GitHub publication / GitHub公开发布

Repository / 仓库：[HoseaZou/EmbodiedGuard](https://github.com/HoseaZou/EmbodiedGuard)
Competition release / 参赛发布版：[competition-v14](https://github.com/HoseaZou/EmbodiedGuard/releases/tag/competition-v14)

The source was published on 2026-10-21. The fixed tag preserves that release; the default branch may add documentation. Do not overwrite the competition tag or existing assets when improving presentation. See [documentation history](DOCUMENTATION-UPDATES.md).

源代码已于2026-10-21公开。固定标签保留首次公开版本，默认分支可补充文档。完善展示时不覆盖参赛标签和已有附件，详见[文档更新记录](DOCUMENTATION-UPDATES.md)。

## Retention / 保留要求

The organizer requested retaining the public repository and its code/files for more than one month. The team's conservative retention date is **at least 2026-11-24**, based on the notice recorded on 2026-10-23, not a separately specified organizer deadline. Keep the repository public and retain competition commits, tag, release and assets. Reaching that date does not authorize automatic deletion or privatization.

组委会要求仓库及代码文件公开保留一个月以上。团队按2026-10-23收到通知保守执行至**至少2026-11-24**，此为内部保守日期，不是组委会另行指定截止日。期间保持公开、保留参赛提交、标签、发布页和附件；到期不自动删除或转私有。

## Maintaining Documentation / 维护文档

Keep `README.md` in English and `README.zh-CN.md` in Chinese with language links. Cover project overview, demo video, architecture and integration standards in both. Documentation changes require a matching `provenance/public-files.json` inventory; do not alter submitted Python files, original hashes or allow-list records to disguise implementation changes.

英文首页与中文版保持语言切换，两版都覆盖项目、视频、架构与集成标准。修改公开说明后同步更新公开文件清单；不改原Python、原始哈希或允许差异记录来掩盖实现变化。

```sh
python tools/verify_release.py
```

`.gitattributes` preserves file bytes. Runtime outputs, personal MCP settings, account-bearing native diagnostics and model credentials remain excluded. `COMPETITION_VERSION.json` and original publication-validation metadata are preparation snapshots; actual publication status is established by the repository, fixed tag, release and CI links above.

`.gitattributes`保护文件字节。运行输出、个人MCP设置、含账号的原生诊断及模型凭据不进入公开仓库；版本文件和旧公开整理验证元数据保留准备阶段状态，实际发布状态以仓库、固定标签、Release及CI为准。
