# Gradle Tasks

## Publishing
This repository uses the dairy-publishing plugin.

This plugin makes it so that you cannot publish an unclean working tree.
(no uncommited or untracked files). Attempts to publish will fail with a warning
about this unclean state.

If you make a copy of this repository, it will fail to sync until you have an
initial commit due to "invalid HEAD state". I recommend making a pretty empty 
initial commit of this template in order to sync, and then ammending it before 
you push anywhere.

Publication versions are taken from the current commit tag. Or, if there isn't
one, then the version will be `SNAPSHOT-<commit hash>` where `<commit hash>` is
replaced with the short commit hash of the current commit.

Snapshot versions will be published to the dairy snapshots repository, tagged
commits will be published to the dairy releases repository.

It is possible to easily modify the publication locations, but if you want to
actually publish to the dairy repository, then reach out to get publication
access.

### `assemble`
'Build's your library (compiles it).

### `build`
Assembles and tests your library.

### `test`
Tests your library.

### `displayVersion`
Will print out the version that is determined from git.

### `publishToMavenLocal`
Publishes to your maven local repository, found at `~/.m2`.

This is good to test publication before you actually do publish.

### `publishReleasePublicationToDairyRepository`
Publishes to the dairy maven repository (https://repo.dairy.foundation/).
