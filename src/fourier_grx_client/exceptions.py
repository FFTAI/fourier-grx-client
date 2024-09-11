class FourierBaseException(Exception):
    pass


class ThreadExitException(FourierBaseException):
    """Trigger a thread to exit."""

    pass


class FourierTimeoutError(TimeoutError):
    pass


class FourierValueError(ValueError):
    pass


class FourierConnectionError(ConnectionError):
    pass
