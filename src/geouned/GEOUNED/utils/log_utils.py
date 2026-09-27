import logging

formatter = logging.Formatter("%(asctime)s :: %(levelname)s :: %(funcName)s :: %(lineno)d :: %(message)s")


def setup_logger(name, log_file, level=logging.DEBUG):
    """To setup as many loggers as you want.

    Loggers are process-global, so whatever an earlier run left on this one (its file handlers, its filters) is
    cleared first: otherwise every new run adds one more handler and each message is written once per previous run
    (to that run's files too)."""

    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    for old_handler in list(logger.handlers):
        logger.removeHandler(old_handler)
        old_handler.close()
    logger.filters.clear()
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger
