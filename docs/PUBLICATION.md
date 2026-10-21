# 发布GitHub时使用

仓库名称建议：`EmbodiedGuard`。描述：`U308参赛版本V14｜基于Agnes Harness的无人巡检自主决策与安全恢复智能体`。

本目录已包含README、许可、来源记录、参赛身份和运行方法。创建Public仓库后上传本目录根内容（不要上传原比赛文件夹或全部提交ZIP）。先运行`python tools/verify_release.py`；安装依赖后运行README的测试命令。

Git操作示例（REPOSITORY_URL替换为实际仓库）：

```sh
git init -b main
git add .
git commit -m "Release EmbodiedGuard competition V14 (U308)"
git tag -a competition-v14 -m "Original competition V14 from r3 delivery; documented publication edits"
git remote add origin REPOSITORY_URL
git push -u origin main
git push origin competition-v14
```

`.gitattributes`禁用字节转换，保护已固定的文件指纹。公开后使用未登录窗口确认README、代码与标签可见；由队长填写飞书队伍信息与实际仓库URL，每队一次。截止日期按组委会通知为2026-10-25；不要填本地目录或尚不存在的URL。

本次整理不自动创建GitHub仓库、不代填飞书，也不在此仓库写入账号或密钥。
