# 上传 GitHub

只上传本目录 `ACG138-FPGA-Robot`，不要把外层工作区、调试资料、密码文件或安装包一起提交。源文件已保留中文说明，未设置全仓开源许可证。建议先建立私有仓库，按 [THIRD_PARTY.md](../THIRD_PARTY.md) 核对第三方文件后再决定公开范围。

本地仓库已经初始化，文件已暂存，但没有创建提交，也没有连接远程仓库。新建一个空 GitHub 仓库后，在本目录执行：

```bash
git status
git commit -m "Initial import: FPGA robot controller and RK viewer"
git remote add origin https://github.com/YOUR_ACCOUNT/YOUR_REPOSITORY.git
git push -u origin main
```

把仓库地址替换成自己的地址。Git 提示身份未配置时，再设置自己的 `user.name` 和 `user.email`；这里没有替你写入姓名或邮箱。

压缩包包含同样的源码，但不包含 `.git`。若从 ZIP 解压，请先在解压目录执行 `git init -b main` 和 `git add .`，再按上面步骤提交。

模型权重、视频、编译位流、许可证和 Python 依赖均不在源码包中。它们不需要为了运行说明而放进 Git 历史。
