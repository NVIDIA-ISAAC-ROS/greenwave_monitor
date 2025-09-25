from textual.reactive import reactive
from textual.scroll_view import ScrollView


class LogLines(ScrollView):
    show_find: reactive[bool] = reactive(False)
    show_line_numbers: reactive[bool] = reactive(False)
