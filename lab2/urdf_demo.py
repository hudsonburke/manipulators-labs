import webbrowser

from roboticstoolbox.models.URDF.URDFRobot import URDFRobot
from swift import Swift


def print_swift_url(url: str, *_args: object, **_kwargs: object) -> bool:
    """Print the remote URL instead of opening a browser inside Codespaces."""
    print(
        "Swift is running. Open the forwarded port 52000 in the Ports panel, "
        "then append /?53000 to its URL:"
    )
    print(f"  {url}")
    return True


# Codespaces cannot display a browser opened inside the development container.
webbrowser.open_new_tab = print_swift_url

ur5 = URDFRobot("ur5")
ur5.q = ur5.random_q()

env = Swift()
env.launch(realtime=True)
env.add(ur5)
env.step()
env.hold()
