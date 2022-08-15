# TODO: Right click menu is randomly slow

# TODO: Select a node, click a text field, press delete. Selected node incorrectly gets deleted

# TODO: File input field opens a weird blank window. Not a pressing issue, but looks funny

# TODO: Image doesn't disappear when there is an error?

# TODO: Node title bar colors

# TODO: Mixing fields with sockets makes socket centers offset

# TODO: Clean up imports

# TODO: Improve socket graphics for high DPI

# TODO: Move socket graphic creation somewhere else

# TODO: Fix edge selection bounds

# TODO: Hide output choice combobox when only one output exists

# TODO: Dragging edge between two outputs or two inputs makes many errors

# TODO: Self connection checking

# TODO: Loop checking

# TODO: Nodes without sockets on one side have small QHLine padding issues on that side

# TODO: Move constants to another python file

# TODO: Optional inputs (color of sockets?)

# TODO: Add a better UI way to delete nodes

# TODO: Find an official way to close the application

# TODO: Figure out if green highlight is foreshadowing problems

# TODO: Understand Node paint() logic

# TODO: Understand Edge paint() logic

# TODO: Add typing information to variables... is this even possible???

# TODO: Add typing information to functions

# TODO: Use global constants for paint colors

# TODO: Make the NodeContainer simply a bundle, all logic should be handled with Node when possible

# TODO: Move fields, inputs, outputs to Node

# TODO: Rename edge to link

# TODO: Rename edge 'startsocket' and 'endsocket' to source/sink

# TODO: Rename fake link to temp link

# TODO: Rename NodeContent to something without 'content'

# TODO: Fix argument name QStyleOptionGraphicsItem in paint functions

# TODO: _init_ui

# TODO: Fancy

# TODO: Make nodes collapsible

# TODO: Add searchable node tray

# TODO: Write function that analyzes an existing function for input/output data

# TODO: Dragging link to widget, but not specific socket, should intelligently guess socket

# TODO: Try out code styling extensions (black)

# TODO: Questions

# TODO: Temp link should remember the whole socket - not just pos - for graphics?

# TODO: Why are edges automatically updating??

# TODO: Are Nodes (QGraphicsItem) supposed to be made with a parent? n = Node(parent) style?

# TODO: After that ^, check _all_ created widgets for QObj(parent) correctness

# NOTE: QApplication.processEvents()

# NOTE: arr2 = np.require(arr, np.uint8, 'C')

# NOTE: {

# "key": "cmd+w"

# "command": "workbench.action.closeWindow"

# "when": "!editorIsOpen && !multipleEditorGroups"

# }

# NOTE: It looks like the QGraphicsItem handles the graphics bit

# the sub-QGraphicsProxyWidget holds the widget contents, and

# those contents are a QWidget

# NOTE: QWidgets under a graphics proxy widget should have a null parent

# NOTE: QPixmap - .setDevicePixelRatio(2)
