let current_mode = "angular";
let current_motor = 1;

const close_connection = () => {
    window.location.href = '/';
};

const change_mode = () => {
    if(current_mode === "angular") {
        current_mode = "cartesian";
    } else {
        current_mode = "angular";
    }
};

const change_motor = () => {
    if(current_motor == 5)
        current_motor = 1;
    else
        current_motor++;
};

let successful = null;

const inc_height = () => {
    successful = fetch_req('inc_height');    
};

const dec_height = () => {
    successful = fetch_req('dec_height'); 
};

const inc_degree = () => {
    successful = fetch_req('inc_degree');
};

const dec_degree = () => {
    successful = fetch_req('dec_degree');
};

const fetch_req = (pathname) => {
    const response = fetch(`/${pathname}`, {
    })
    .then(response => response.json())
    .then(data => {
        if(data['successful' !== true])
        {
            return false;
        } else {
            return true;
        }
    })
    .catch(err => false);
    return response
}