def format_urdf_name(name: str) -> str:
    ''' Format a string to be a valid URDF name

    Parameters
    ----------
    name : str
        input name

    Returns
    -------
    str
        formatted name
    '''
    return name.replace(':','_').replace(' ','').replace('-','_').lower()