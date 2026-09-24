import uvicorn

from aster_game.app.config import get_settings


def main() -> None:
    settings = get_settings()
    uvicorn.run(
        "aster_game.app.main:app",
        host=settings.host,
        port=settings.port,
        log_level=settings.log_level.lower(),
        ws_max_size=16_384,
    )


if __name__ == "__main__":
    main()
