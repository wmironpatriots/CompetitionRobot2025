# CODEBASE NAME
<!-- Quick and dirty description of the codebase -->
FRC team 6423's !!YEAR!! robot code for !!GAME-NAME!!

###### README Overview
<!-- README table of Contents-->
* [Contribution Guidelines](#contribution-guidelines)
* [Setup Guide](#setup-guide)
* [Basic Gradlew Tasks](#basic-gradlew-tasks)
* [Licenses](#licenses)
---

## Contribution Guidelines
<!-- Import contribution requirements -->
All contributions must be made under the terms of the Developer Certificate of Origin 1.1

```
By making a contribution to this project, I certify that:

(a) The contribution was created in whole or in part by me and I
    have the right to submit it under the open source license
    indicated in the file; or

(b) The contribution is based upon previous work that, to the best
    of my knowledge, is covered under an appropriate open source
    license and I have the right under that license to submit that
    work with modifications, whether created in whole or in part
    by me, under the same open source license (unless I am
    permitted to submit under a different license), as indicated
    in the file; or

(c) The contribution was provided directly to me by some other
    person who certified (a), (b) or (c) and I have not modified
    it.

(d) I understand and agree that this project and the contribution
    are public and that a record of the contribution (including all
    personal information I submit with it, including my sign-off) is
    maintained indefinitely and may be redistributed consistent with
    this project or the open source license(s) involved.
```

To show that you've accepted the terms, **ensure that all your commits are signed off**
```
Signed-off-by: Bob R Smith <bob.r.smith@developer.example.org>
```
You can generate this message in your commits by using the `-s` arguement: `git commit -m "message" -s`. There is also a git hook that can be installed to signoff for you if you forget.

To maintain code quality and prevent commits on default branch, **make sure you install pre-commit hooks before commiting anything**

---

## Setup Guide
<!-- Basic workspace setup guide -->
#### Prerequisites
* Java-Development-Kit (JDK) 17 or higher
* An IDE
    * **[WPIlib VScode (RECOMMENDED)](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)**
    * **[IntelliJ](https://www.jetbrains.com/idea/)**

#### Steps
1. Clone this repository !!REPO-LINK!!
2. Run `./gradlew` to download gradlew and all WPIlib/Vendor dependancies
3. Run `./gradlew installGitHooks` to install hooks
4. If you're using IntelliJ, run `./gradlew idea` and open the generated `.ipr` file in IntelliJ
4. You're all set!

---

## Basic Gradlew Tasks
<!-- Commonly used gradlew tasks-->
* Run `./gradlew build` to build code
* Run `./gradlew assemble` to build code without running JUnit tests
* Run `./gradlew test` to run all JUnit tests
* Run `./gradlew deploy` on the robot network to upload robot code to rio

Run `./gradlew tasks` to see other runable tasks

To find out more about a task, run `./gradlew [TASK] --info`

---

## Packages
<!-- Description of all packages in src -->
<!-- TODO -->

---

## Licenses
<!-- All licenses for project software -->
* FRC team 6423's code: [`LICENSE`](./LICENSE)
* WPIlib: [`WPILib-License.md`](./WPILib-License.md)
