from abc import ABC, abstractmethod
from threading import Event, Thread


class WatcherBase(ABC):
    def __init__(self) -> None:
        self._thread: Thread | None = None
        self._exit_event = Event()
        super().__init__()

    def start(self) -> None:
        assert self._thread is None
        self._thread = Thread(target=self.run, name=repr(self))
        self._thread.start()

    def close(self) -> None:
        if not self._exit_event.is_set():
            self._exit_event.set()
            self._thread = None

    @abstractmethod
    def run(self) -> None:
        """Thread runner."""
