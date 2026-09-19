from rich.console import Console
from rich.logging import RichHandler
from rich.theme import Theme
import logging

THEME = Theme({
    "info": "cyan",
    "warning": "yellow",
    "error": "bold red",
    "success": "bold green",
    "robot": "magenta",
    "pipeline": "blue",
    "input": "green",
    "render": "yellow",
})

console = Console(theme=THEME)

logging.basicConfig(
    level=logging.INFO,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[RichHandler(
        console=console,
        rich_tracebacks=True,
        tracebacks_show_locals=True,
        markup=True,
    )],
)

logger = logging.getLogger("mink_teleop")
logger.setLevel(logging.DEBUG)

def log_info(msg: str, tag: str = "INFO"):
    logger.info(f"[{tag}] {msg}")

def log_success(msg: str, tag: str = "SUCCESS"):
    logger.info(f"[bold green][{tag}][/] {msg}")

def log_warning(msg: str, tag: str = "WARNING"):
    logger.warning(f"[yellow][{tag}][/] {msg}")

def log_error(msg: str, tag: str = "ERROR"):
    logger.error(f"[bold red][{tag}][/] {msg}")

def log_robot(msg: str):
    logger.info(f"[magenta][ROBOT][/] {msg}")

def log_pipeline(msg: str):
    logger.info(f"[blue][PIPELINE][/] {msg}")

def log_input(msg: str):
    logger.info(f"[green][INPUT][/] {msg}")

def log_render(msg: str):
    logger.info(f"[yellow][RENDER][/] {msg}")

def log_debug(msg: str, tag: str = "DEBUG"):
    logger.debug(f"[dim][{tag}][/] {msg}")
