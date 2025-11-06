"""Run flake8 test."""
import warnings
from pathlib import Path

from ament_flake8.main import main


def test_flake8():
    """Run test using pytest."""
    repo_root = Path(__file__).resolve().parents[1]
    cfg = repo_root / '.flake8'
    with warnings.catch_warnings():
        warnings.filterwarnings('ignore', '.*multi-threaded.*fork.*')
        assert main(argv=['--config', str(cfg)]) == 0
