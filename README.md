# TicTacToeAR4

## Running the Nodes

### AI Node
`ros2 run tictacai exec`

### Vision Node
`ros2 run tictacvis exec`
To trigger a board update, reset the board, make a move manually, etc., use the `user_control.py` script.
To update the corners for the vision node, use the `getcorners.py` script.


Generally, you will want to run the AI node and the Vision node together before you begin calling for updates.