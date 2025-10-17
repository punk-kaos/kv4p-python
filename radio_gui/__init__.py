"""Interactive GUI for controlling a KV4P radio."""

from typing import TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - import-time hinting only
    from .app import RadioApp as _RadioApp


def run(*, debug: bool = False) -> None:
    """Launch the GUI application."""

    from .app import run as _run

    _run(debug=debug)


__all__ = ["run"]
