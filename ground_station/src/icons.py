import qtawesome as qta
from PyQt5.QtGui import QIcon
from types import SimpleNamespace


def get_icons() -> SimpleNamespace:
    """
    Load and return a set of icons for the application.

    Returns
    -------
    SimpleNamespace
        A namespace object containing the loaded icons.
        Each icon can be accessed as an attribute of the namespace.

        Example: `icons.upload` = `icons["upload"]`

    Raises
    -------
    TypeError
        If an icon fails to load or is not a QIcon.
        This indicates that the icon name is not valid or the icon could not be found.
    """

    icons = {
        "upload": qta.icon("mdi.upload"),
        "download": qta.icon("mdi.download"),
        "delete": qta.icon("mdi.trash-can"),
        "save": qta.icon("mdi.content-save"),
        "edit": qta.icon("mdi.cog"),
        "refresh": qta.icon("mdi.refresh"),
        "hard_drive": qta.icon("fa6.hard-drive"),
        "boat": qta.icon("mdi.sail-boat"),
        "image_upload": qta.icon("mdi.image-move"),
    }

    for icon_name, icon in icons.items():
        if not isinstance(icon, QIcon):
            raise TypeError(f"Failed to load icon: {icon_name} is not a QIcon")

    return SimpleNamespace(**icons)
