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
    def __init__(self, prefix: str, config: zenoh.Config = zenoh.Config()):
        self.prefix = prefix
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
        callback: Callable[[zenoh.Reply], Any] | None = None,
    ):
        def _callback(reply: zenoh.Reply):
            try:
                res = Serde.unpack(reply.ok.value.payload)
                logger.debug(f"{topic}: {res}")
            except Exception:
                # TODO: handle error
                logger.error(f"{topic}: {reply.err.payload.decode()}")

        self.session.get(
            self._encode_topic(topic, parameters),
            callback if callback else _callback,
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
        # print(self._encode_topic(topic, parameters),self._encode_value(value),self._encode_attachment(attachment),)
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
            reply = receiver.get(timeout)
            # print(reply)
        except TimeoutError as ex:
            raise TimeoutError(f"Timeout waiting for {topic}") from ex
        except StopIteration:
            logger.debug(f"No reply from {topic}")
            return None

        try:
            # print(reply.ok.value.payload)
            res = Serde.unpack(reply.ok.value.payload)
            logger.debug(f"{topic} ({value}): {res}")
            return res
        except Exception:
            err_msg = reply.err.payload.decode()
            logger.error(f"{topic} ({value}): {err_msg}")
            return None

    def _publish(self, topic: str, value: Any, attachment: dict[str | bytes, Any] = {}):
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
        # print(self._publishers, self._subscribers, self._services)
        try:
            for q in self._services.values():
                # print(f"Undecalare service {q.key_expr}")
                q.undeclare()
            for s in self._subscribers.values():
                # print("Undecalare subscribers")
                s.undeclare()
            for p in self._publishers.values():
                # print(f"Undecalare publisher {p.key_expr}")
                p.undeclare()
        except Exception as ex:
            print(ex)
            pass
        try:
            if self.session:
                self.session.close()
            self.session = None
        except Exception:
            pass

    def __del__(self):
        self.close()
