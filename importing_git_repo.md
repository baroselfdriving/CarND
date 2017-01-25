# How to import existing Git repository into another

Projects and labs are provided as git repositories in this course. The following steps describe how I imported these 
repositories into my personal course repository __CarND__.

* My course repository: https://github.com/cvilas/CarND
* Example Udacity project repository: https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project

```
git remote add other https://github.com/udacity/CarND-Traffic-Sign-Classifier-Project
git fetch other
git checkout -b P2-TrafficSigns other/master
mkdir P2-TrafficSigns
git mv stuff P2-TrafficSigns/stuff             # repeat as necessary for each file/dir
git commit -m "Moved stuff to P2-TrafficSigns"
git checkout master 
git merge P2-TrafficSigns                      # should add ZZZ/ to master
git commit
git remote rm other
git branch -d P2-TrafficSigns                  # to get rid of the extra branch before pushing
git push                           			   # if you have a remote, that is
```

# References

* [This StackOverflow post] (http://stackoverflow.com/questions/1683531/how-to-import-existing-git-repository-into-another)

