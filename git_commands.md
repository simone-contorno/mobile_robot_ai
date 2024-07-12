# Git base commands

## Create new branch from the main one

Go in the main branch:
<pre><code>git checkout main</code></pre>

Update the main branch:
<pre><code>git fetch origin main</code></pre>
<pre><code>git merge origin/main</code></pre>

OR

<pre><code>git pull origin main</code></pre>
Note: pull is fetch and merge together but there is less control over possible conflicts.

Create the new branch: 
<pre><code>git checkout -b [new-branch]</code></pre>

## Commit 

<pre><code>git commit [file(s) or '-a' for all] -m "[message]"</code></pre>

## Merge

When no new commits have been made on the base branch since the feature branch diverged from it, Git will perform a Fast-Forward Merge (no conflicts check) which could overwrite your code.

Merge:
<pre><code>git merge [branch]</code></pre>

If there are conflicts, the merge can be abort:
<pre><code>git merge --abort</code></pre>

## Uncommit / Unmerge

List commits:
<pre><code>git log --oneline</code></pre>
--oneline flag will list every commmits.

Revert without modifying the commit history:
<pre><code>git revert -m 1 [commit-hash]</code></pre>

OR

Reset to the commit history point you want to:
<pre><code>git reset [commit-hash]</code></pre>
Note: using the flag '--hard' will also remove the changes made by the undone commits.

