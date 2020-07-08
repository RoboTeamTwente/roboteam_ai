Here is some general non-technical advise when working on the code base, which is applicable for any software developer at any company. When writing code you should take the following points into account:

# Documentation (!= Comments)
## Why?
Code base documentation is very important when writing code, because:
* It makes it easier for other software developers to review your pull requests, since they only have to check if the code corresponds with the specification (documentation) and check if the specification is suitable. Without documentation reviewers have to figure out the specification themselves which takes more time to review the pull request.
* It makes it easier to find bugs in the code, since you just have to compare the documentation with the implementation. If the documentation does not match the implementation then the implementation is supposed to be wrong. Without documentation you have no clue whether something in the code has been implemented intentionally or unintentionally.
* It stimulates re-usage, without documentation other software developers have problems figuring out which functionality is provided by which methods, what preconditions are required and which post-conditions have been satisfied.
* It makes it easier to adhere to the other points: Design, Testing, Consistency, Code cloning/Dead code. For example without documentation it is not even possible to test your code, because you have no specification which you compare the implementation against.
* Your code could stay in the RoboTeam repositories for more than 5 years, when you are long gone from the RoboTeam. The software developers in that team however should understand your code. And every year that passes knowledge about the code base is lost which is kept by proper documentation.

## How?
Keep in mind that documentation is different than comments. Comments are placed between code lines in the source file, whereas documentation is placed in the header file before function and class definitions. Good code should be self-explanatory which means that it often does not need any comments, when properly split into smaller functions that have been well-documented. And when writing comments, it should mostly be used to answer the why questions rather than the what questions, i.e. comments should focus on why does this part of code works, why don't I have to check certain situations, etc. rather than what the code actually does. Documentation on the other hand can be used to explain what the code does, but should also be used to explain why this code works. Documentation should give an answer to the most important questions regarding the code which can include answers on questions like:
* What does the code compute? What is the meaning of the output? What are the postconditions on the output? What is the meaning of the input arguments? Which preconditions should hold on the input arguments? Which requirements should be satisfied before using the code? Etc.
* Is there some known bug/strange behavior in the code which hasn't been solved yet? How does the code respond in case of an exception? What happens when preconditions/requirements have been violated? Etc.
* What is the meaning of abbreviations used inside the code? Is there a website which explains (partially) the meaning of the code? From which source has this code been copied? Etc.
* In case of class documentation: Who has written/maintained this code? At which date has this code been written?
* Less important, but could be included as well: Why does this code work? Why is this code efficient? Why did you choose this type of design? What other solutions have you tried out for this code? What major decisions have been made regarding this code? 

## Good Example
Below is an example how you should document your code:
```
/**
 * Get the intersection between two (finite) LineSegment. Note that this function knows how to deal with LineSegments that are points and how to deal with
 * parallel LineSegments. Moreover this function guarantees to return an intersection point if the LineSegments intersects. In case:
 * - There is no intersection then std:nullopt is returned.
 * - There is a single intersection point then that intersection point is returned.
 * - There are multiple intersection points (infinitely many) which happens when the LineSegments overlap then it prefers to returns 1: start, 2: end, 3: 
 *  line.start of which the first most option is selected that lies on both LineSegments. Note that if none of these points lie on both LineSegments and if both 
 *  the LineSegments are parallel then the LineSegments cannot intersect.
 *
 * @param line The other (finite) LineSegment.
 * @return std::nullopt if the LineSegments do not intersect. Otherwise return an intersection point.
 */
[[nodiscard]] std::optional<Vector2> intersects(const LineSegment &line) const;
```
Note that this documentation gives a clear description of what the code does, by explaining what happens for each situation. But besides that it also explains why this code works, by telling that the function knows how to deal with parallel & point cases and that you don't have to check if line.end is on the line if start, end and line.start are not on both lines.

## Bad Example
Below is an example how you should not document your code:
```
/**
 * Calculates info for the defenders
 */
void calculateInfoForDefenders() noexcept;
```
The documentation basically tells the same as the name of the function. Hence the goal of documentation, to inform others about usage, is missing here. What kind of info is computed is totally unclear? What are requirements for calling this function is also missing? When should this function be called?

# Design

# Unit Testing
## Why?
Writing unit test cases for your code is very important, because
- Even the simplest code can contain bugs and even when fully reasoned that the code works, it can still contain some bugs.
- It encourage you to inspect your code better for bugs, because during the process of writing test you might already find bugs (without executing the tests).
- It improves the trust in the code, since it is properly tested and therefore it is less likely that your code suddenly fails during the RoboCup.
- It will help you find the location of bugs.

## How?
When writing unit test cases, your goal is to create unit test cases that might/will fail. So for example you should test the path planning algorithm for the case that the ball cannot be reached (because it is surrounded by enemy robots) and for LineSegments you should test LineSegments that are actually points (have an equal start position as end position). Using random unit test cases or just writing a few test cases will in most cases not found the most common type of bugs. Hence you should be more systematic in writing test cases. For this there are 2 types of approaches: black box testing and white box testing.

# Consistency

# Dead code & Code clones
