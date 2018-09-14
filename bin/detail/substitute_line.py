import io
import re

def substitute_line(filename, pattern, repl):
  with io.open(filename, "r") as f:
    lines = f.readlines()
  with io.open(filename, "w", newline='\n') as f:
    for line in lines:
      f.write(re.sub(pattern, repl, line))
