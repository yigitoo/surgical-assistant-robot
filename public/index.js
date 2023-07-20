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
    if(current_mode === "angular") return;
    successful = fetch_req('inc_height', current_motor);    
};

const dec_height = () => {
    if(current_mode === "angular") return;
    successful = fetch_req('dec_height', current_motor); 
};

const inc_degree = () => {
    if(current_mode !== "angular") return;
    successful = fetch_req('inc_degree', current_motor);
};

const dec_degree = () => {
    if(current_mode !== "angular") return;
    successful = fetch_req('dec_degree', current_motor);
};

const fetch_req = (pathname, current_motor) => {
    const response = fetch(`/${pathname}/${current_motor}`, {
        method: 'GET',
        headers: {
            'Content-Type': 'application/json'
        }
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