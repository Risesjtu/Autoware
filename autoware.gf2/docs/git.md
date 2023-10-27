# Git


# 怎么写 Git 提交日志?
## 规范
> 在一个团队协作的项目中，开发人员需要经常提交一些代码去修复bug或者实现新的feature。而项目中的文件和实现什么功能、解决什么问题都会渐渐淡忘，最后需要浪费时间去阅读代码。但是好的日志规范commit messages编写有帮助到我们，它也反映了一个开发人员是否是良好的协作者。

编写良好的 Commit messages 可以达到3个重要的目的
- 加快代码 review 的流程
- 帮助开发人员编写良好的版本发布日志
- 让之后的维护者了解代码里出现特定变化和 feature 被添加的原因

本项目建议使用 [`Angular` 规范](https://github.com/angular/angular.js/blob/master/DEVELOPERS.md#commits) 来编写提交日志, 主要由 `header`, `body` 和 `footer` 三部分组成. 其中 `header` 包括 `type` 和 `subject`, 示例如下.  
```md
<type>: <subject>

<body>

<footer>
```

- `header`: 应当尽量简短, 描述重要更新.
  - `type`: 本次提交的类型, 可从如下类型中选择一个:  
    ```md
    # 主要type
    feat: 增加新功能
    fix: 修复bug
     
    # 特殊type
    docs: 只改动了文档相关的内容
    style: 不影响代码含义的改动，例如去掉空格、改变缩进、增删分号
    build: 构造工具的或者外部依赖的改动，例如webpack，npm
    refactor: 代码重构时使用
    revert: 执行git revert打印的message
     
    # 暂不使用type
    test: 添加测试或者修改现有测试
    perf: 提高性能的改动
    ci: 与CI（持续集成服务）有关的改动
    chore: 不修改src或者test的其余修改，例如构建过程或辅助工具的变动
    ```
  - `subject`: 简明扼要的阐述下本次 commit 的主旨, 是 commit 目的的简短描述, 建议不超过50个字符, 遵循如下规范: 
    - 使用祈使句与一般现在时: "change" not "changed" nor "changes"
    - 首字母不要大写
    - 句尾不要标点

- `body`: 同样使用祈使句, 一般现在时. 在主体内容中我们需要把本次 commit 详细的描述一下, 比如此次变更的动机, 以及变更的内容.

- `footer`: 如果本次提交解决了 Github 上的某个 issue, 可以在这里关闭. 如果本次提交包含不兼容的更新, 则在此处标示并说明, 例如
  ```md
  BREAKING CHANGE: [the rest commit message]
  ```
  或者
  ```md
  BREAKING CHANGE: 

  [the rest commit message]
  ```

## 提交方式
如果一个 Git 提交日志按照上述规范编写, 就不适宜在命令行中通过 `git commit -m "commit message"` 的方式进行提交了.   

实际上, git 允许我们将提交日志写在一个文件中, 然后指定该文件作为提交日志进行提交: `git commit -F COMMIT_EDITMSG.md`.  

另一种更加简单的方式是在 VS Code 中添加更改后点击提交，将自动打开 COMMIT_EDITMSG 编辑窗口来写入提交日志，写入完毕后保存并关闭该窗口即可完成提交.  

更多 git 使用方法可参考   
- [Git 知识大全 - Gitee](https://gitee.com/help/categories/43).
- [Git Cheat Sheet 中文版 - Github](https://github.com/flyhigher139/Git-Cheat-Sheet)



