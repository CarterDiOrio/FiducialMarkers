# Mount Fiducial File Format

## Transforms
- $T_{mp}$ : Transform between the paper and the top left marker

## Format

- Line 1: "Fiducial Name", csv fiducial parameters
- Line 2: $T_{mp}$ elements from top left to bottom right as comma seperated values
- Line 3-N: X, Y corner location in the paper coordinate system. One per lined