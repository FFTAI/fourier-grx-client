from __future__ import annotations

from collections.abc import Callable
from typing import Any

import zenoh
from loguru import logger

from .utils import Serde


def parse_reply(reply: zenoh.Reply) -> tuple[bool, Any]:
    if reply.is_ok:
        return True, Serde.unpack(reply.ok.value.payload)
    else:
        return False, reply.err.payload.decode()


class ZenohSession:
    def __init__(self, prefix: str, config: zenoh.Config | None = None):
        self.prefix = prefix

        if config is None:
            config = zenoh.Config()
        self.session: zenoh.Session = zenoh.open(config)
        self._publishers: dict[str, zenoh.Publisher] = {}
        self._subscribers: dict[str, zenoh.Subscriber] = {}
        self._services: dict[str, zenoh.Queryable] = {}

    def _encode_attachment(self, attachment: dict[str | bytes, Any]) -> dict[str | bytes, str | bytes]:
        encoded_attachment = {k: Serde.pack(v) for k, v in attachment.items() if v is not None}
        return encoded_attachment  # type: ignore

    def _encode_value(self, value: Any) -> bytes | None:
        if value is not None and not isinstance(value, bytes):
            encoded_value: bytes | None = Serde.pack(value)
        else:
            encoded_value = value
        return encoded_value

    def _encode_topic(self, topic: str, parameters: dict[str, Any]) -> str:
        parameters_str = "&".join([f"{k}={v}" for k, v in parameters.items() if v is not None])
        if parameters_str:
            if "?" not in topic:
                topic = f"{topic}?{parameters_str}"
            else:
                topic = f"{topic}&{parameters_str}"
        return f"{self.prefix}/{topic}"

    def _call_service(
        self,
        topic: str,
        value: Any | None = None,
        parameters: dict[str, Any] = {},
        attachment: dict[str | bytes, str | bytes] = {},
        timeout: float = 1.0,
        callback: Callable[[Any], Any] | None = None,
    ):
        def _callback(reply: zenoh.Reply):
            try:
                res = Serde.unpack(reply.ok.value.payload)
                logger.debug(f"{topic}: {res}")
                if callback:
                    callback(res)
            except Exception:
                # TODO: handle error
                logger.error(f"{topic}: {reply.err.payload.decode()}")

        self.session.get(
            self._encode_topic(topic, parameters),
            _callback,
            value=self._encode_value(value),
            attachment=self._encode_attachment(attachment),
            timeout=timeout,
        )
        return

    def _call_service_wait(
        self,
        topic: str,
        value: Any | None = None,
        parameters: dict[str, Any] = {},
        attachment: dict[str | bytes, Any] = {},
        timeout: float = 15.0,
    ):
        if self.session is None:
            logger.warning("Session is closed")
            return
        receiver = self.session.get(
            self._encode_topic(topic, parameters),
            zenoh.Queue(),
            value=self._encode_value(value),
            attachment=self._encode_attachment(attachment),
            timeout=timeout,
        )

        try:
            # TODO: allow this to receive multiple replies
            reply = receiver.get(timeout)
        except TimeoutError:
            logger.warning(f"Timeout waiting for reply from {topic}")
            return None
        except StopIteration:
            logger.debug(f"No reply from {topic}")
            return None
        finally:
            receiver.close()

        try:
            # print(reply.ok.value.payload)
            res = Serde.unpack(reply.ok.value.payload)
            logger.debug(f"{topic} ({value}): {res}")
            return res
        except Exception:
            err_msg = reply.err.payload.decode()
            logger.error(f"{topic} ({value}): {err_msg}")
            return None

    def _call_action(
        self,
        topic: str,
        value: Any | None = None,
        parameters: dict[str, Any] = {},
        attachment: dict[str | bytes, Any] = {},
        timeout: float = 15.0,
    ):
        if self.session is None:
            logger.warning("Session is closed")
            return
        receiver = self.session.get(
            self._encode_topic(topic, parameters),
            zenoh.Queue(),
            value=self._encode_value(value),
            attachment=self._encode_attachment(attachment),
            timeout=timeout,
        )
        try:
            for reply in receiver:
                ok, res = parse_reply(reply)
                if ok:
                    logger.debug(f"{topic} ({value}): {res}")
                    yield res
                else:
                    logger.error(f"{topic} ({value}): {res}")
                    yield None
        except TimeoutError:
            logger.warning(f"Timeout waiting for reply from {topic}")
        except StopIteration:
            logger.debug(f"Reply from {topic} completed")
        finally:
            receiver.close()

    def _publish(self, topic: str, value: Any, attachment: dict[str | bytes, Any] | None = None):
        if attachment is None:
            attachment = {}
        if topic not in self._publishers:
            raise ValueError(f"Unknown topic: {topic}")
        if self._publishers[topic] is None:
            logger.warning(f"Publisher {topic} is None")
            return
        try:
            self._publishers[topic].put(self._encode_value(value), attachment=self._encode_attachment(attachment))
        except Exception as ex:
            logger.debug(f"Error publishing to {topic}: {ex}")

    def close(self):
        logger.info("Closing session...")
        try:
            logger.trace("Undecalare services")
            for q in self._services.values():
                q.undeclare()
            logger.trace("Undecalare subscribers")
            for s in self._subscribers.values():
                s.undeclare()
            logger.trace("Undecalare publishers")
            for t, p in self._publishers.items():
                logger.trace(f"Undecalare publisher on topic: {t}")
                p.undeclare()
        except Exception as ex:
            logger.exception(ex)
        try:
            if self.session:
                self.session.close()
            self.session = None  # type: ignore
        except Exception:
            pass

    def __del__(self):
        if self.session:
            self.close()
