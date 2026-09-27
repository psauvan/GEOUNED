import logging

from geouned.GEOUNED.utils.log_utils import setup_logger


def test_a_new_run_starts_the_logger_clean(tmp_path):
    first = tmp_path / "first.log"
    second = tmp_path / "second.log"
    name = "geouned_test_logger"
    try:
        logger = setup_logger(name, first)
        logger.info("run one")
        logger = setup_logger(name, second)
        logger.info("run two")
        assert len(logger.handlers) == 1
        for handler in logger.handlers:
            handler.flush()
        assert first.read_text().count("run one") == 1
        assert "run two" not in first.read_text()  # the previous run's file is no longer written to
        assert second.read_text().count("run two") == 1
        assert "run one" not in second.read_text()
    finally:
        for handler in list(logging.getLogger(name).handlers):
            logging.getLogger(name).removeHandler(handler)
            handler.close()
