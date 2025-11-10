from abc import ABC, abstractmethod

class ICommandDispatcher(ABC):
    
    @abstractmethod
    def arm(self) -> None:
        pass

    @abstractmethod
    def take_off(self) -> None:
        pass

    @abstractmethod
    def offboard(self) -> None:
        pass

    @abstractmethod
    def go_to(self) -> None:
        pass

    @abstractmethod
    def return_to_launch(self) -> None:
        pass
    
    @abstractmethod
    def land(self) -> None:
        pass

    

    
    
