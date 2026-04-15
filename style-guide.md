# Style Guide

* This is a list of suggested coding practices for the repo.
* Take with a grain of salt, and feel free to edit.
* This will hopefully reduce bugs, miscommunication, and _wasted hours of your life_.
* The first suggestion is to _thoroughly_ read the [Putting Rigid Bodies to Rest paper](https://hbaktash.github.io/files/rolling_dragons_paper.pdf) and [Geometry Central docs](https://geometry-central.net).
  * Take your time with these. You will be re-reading them frequently.

## Readability

* Use `camelCase` for variable names, `PascalCase` for class names
* Prefix private variable names with underscores (e.g. `_var`).
* Add comments where possible
* Add spaces between lines of code
* Describe your changes when creating commits
  * Try a `[action] [object]` format, e.g. `Add class, remove struct, modify vars`

## Maintainability

* **!! DO NOT COMMIT TO MAIN BRANCH WITHOUT JOINT CODE REVIEW !!**
* Make sure code runs before committing
* Avoid committing build/config files + dirs by editing the `.gitignore` accordingly
* Use [code tags like TODO and FIXME](https://en.wikipedia.org/wiki/Comment_(computer_programming)#code-tag) to stub future work
  * See if your IDE has a tag-highlighting plugin
    * Here's a [good one for QtCreator](https://doc.qt.io/qtcreator/creator-reference-to-do-entries-view.html)
    * Here's a [good one for VSCode](https://marketplace.visualstudio.com/items?itemName=Gruntfuggly.todo-tree)

## Optimization

* Try out [bithacks](https://graphics.stanford.edu/~seander/bithacks.html) to speed up code (don't forget to add comments)
* To decide between classes, structs, and namespaces:
  * Classes = funcs + data (persistent state to maintain)
  * Structs = data (with a few lightweight helper funcs)
  * Namespaces = funcs only (or to avoid ownership clashes)

УДАЧИ ВАМ!
