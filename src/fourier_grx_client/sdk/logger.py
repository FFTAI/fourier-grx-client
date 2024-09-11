try:
    from fourier_core.logger import Logger
except ImportError:
    from loguru import logger


    class Logger:
        def __new__(cls, *args, **kwargs):
            if not hasattr(cls, '_instance'):
                cls._instance = super().__new__(cls)

            return cls._instance

        def print_debug(self, message: str):
            logger.debug(message)

        def print_info(self, message: str):
            logger.info(message)

        def print_warning(self, message: str):
            logger.warning(message)

        def print_error(self, message: str):
            logger.error(message)

        def print_success(self, message: str):
            logger.success(message)
