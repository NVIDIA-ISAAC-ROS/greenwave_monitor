from textual.widget import Widget
from textual.widgets import Static

LOGO = r"""       ________                                      
_______\_____  \   ______    .__        ______  _  __
\_  __ \/  ____/  /  ___/  __|  |___   / ___\ \/ \/ /
 |  | \/       \  \___ \  /__    __/  / /_/  >     / 
 |__|  \_______ \/____  >    |__|     \___  / \/\_/  
               \/     \/             /_____/         """  

class Header(Widget):
    DEFAULT_CSS = """
    Header {
      border: none;
      border-title-align: center;
      grid-columns: 20 1fr 60;
      height: 8;
      grid-size: 3;
      layout: grid;
    }

    .header-box {
      margin: 0 0 0 3;
      text-align: left;
    }

    .header-right {
        color: #76b900;
    }
    """

    def left(self):
        return Static("", classes="header-box")

    def center(self):
        return Static("", classes="header-box")

    def right(self):
        return Static(LOGO, classes="header-box header-right")

    def compose(self):
        yield self.left()
        yield self.center()
        yield self.right()
