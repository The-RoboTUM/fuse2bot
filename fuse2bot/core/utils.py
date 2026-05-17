def format_urdf_name(name: str) -> str:
    """Format a string to be a valid URDF name

    Parameters
    ----------
    name : str
        input name

    Returns
    -------
    str
        formatted name
    """
    name = name.replace(":", "_").replace(" ", "").replace("-", "_").lower()
    # also replace ending with "v28_1" with "_1"
    name = re.sub(r"v\d+(_\d+$)", r"\1", name)
    return name
