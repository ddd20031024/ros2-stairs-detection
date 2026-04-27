# Git 常用指令（单人简化版）

## 1) 首次初始化（仅一次）

```bash
cd /home/xpy/stair_chapter2
git init
git branch -M main
git remote add origin https://github.com/ddd20031024/ros2-stairs-detection.git
```

## 2) 首次提交并推送（仅一次）

```bash
git add .
git commit -m "init: first commit"
git push -u origin main
```

## 3) 日常最常用（每天就这几个）

```bash
git status
git add .
git commit -m "你的修改说明"
git pull origin main
git push origin main
```

## 4) 查看历史

```bash
git log --oneline -20
```

## 5) 查看远端

```bash
git remote -v
```

## 6) 常见问题

### push 被拒绝（远端有历史）

```bash
git fetch origin
git pull origin main --allow-unrelated-histories
git add .
git commit -m "merge: sync local with remote"
git push origin main
```

### 提交前检查改动

```bash
git status
git diff
```
