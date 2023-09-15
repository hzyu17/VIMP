#!/bin/bash

# Check if filename is provided
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <filename>"
    exit 1
fi

file="$1"

# Delete the first two lines and last two lines
sed -i '1,2d' "$file"        # Delete first two lines
sed -i '$d' "$file"         # Delete last line

# Left shift content four spaces (assuming spaces are used for indentation)
sed -i 's/^    //' "$file"

echo "File has been processed."
